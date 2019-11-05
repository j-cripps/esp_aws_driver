/**
 * @file ma_jobAgent.c
 *
 * @brief Interacts with the AWS Jobs API to pull any relevant jobs for this Thing,
 * parse them and execute the job.
 *
 * @author Jack Cripps // jackc@monitoraudio.com
 * @date 11/09/2019
 */


#include "ma_jobAgent.h"

#include "ma_jobOTA.h"

#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <unistd.h>
#include <limits.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "driver/gpio.h"
#include "esp_ota_ops.h"
#include "esp_http_client.h"
#include "esp_https_ota.h"
#include "esp_flash_partitions.h"
#include "esp_partition.h"
#include "nvs.h"
#include "nvs_flash.h"

#include "jsmn.h"

#include "aws_iot_config.h"
#include "aws_iot_log.h"
#include "aws_iot_version.h"
#include "aws_iot_mqtt_client_interface.h"
#include "aws_iot_jobs_interface.h"


/* -------------------------------------------------------------------------- */


#define COUNT_OF(x) ((sizeof(x)/sizeof(0[x])) / ((size_t)(!(sizeof(x) % sizeof(0[x])))))


/* -------------------------------------------------------------------------- */


#define EXAMPLE_WIFI_SSID "Monitor Audio WiFi"
#define EXAMPLE_WIFI_PASS "des1gnf0rs0und"


/* -------------------------------------------------------------------------- */

typedef struct jobStruct {
    char jobId[32];
    char jobDoc[256];
    char jobFailMsg[256];
} jobDefinition_t;

/* -------------------------------------------------------------------------- */


static const char *TAG = "ma_JobAgent";

const char *THING_NAME = "ESP32TestThing";

/**
 * @brief Event group to signal when we are connected & ready to make a request
 */
EventGroupHandle_t taskEventGroup;

const int WIFI_PROVISIONED_BIT = BIT0;      /**< Whether the task has been provided with WiFi credentials */
const int WIFI_CONNECTED_BIT = BIT1;        /**< Whether WiFi stack is connected */
const int RESTART_REQUESTED_BIT = BIT2;     /**< Whether task requests restart of unit (usually after successful OTA update) */
const int WORK_COMPLETED_BIT = BIT3;        /**< Task completed all work so can be stopped */
const int JOB_READY_BIT = BIT4;             /**< Whether job ready to be processed */
const int JOB_IN_PROGRESS_BIT = BIT5;       /**< Whether an AWS job is currently being processed */
const int JOB_COMPLETED_BIT = BIT6;         /**< AWS job stage completed flag */
const int JOB_FAILED_BIT = BIT7;            /**< Generic job failed flag */
const int UPDATE_FAILED_BIT = BIT8;         /**< Software update failed flag */


/**
 * @brief CA Root certificate
 *        device ("Thing") certificate
 *        device ("Thing") key
 *        s3 root cert for connecting to AWS S3
 */
extern const uint8_t aws_root_ca_pem_start[] asm("_binary_aws_root_ca_pem_start");
extern const uint8_t aws_root_ca_pem_end[] asm("_binary_aws_root_ca_pem_end");
extern const uint8_t certificate_pem_crt_start[] asm("_binary_certificate_pem_crt_start");
extern const uint8_t certificate_pem_crt_end[] asm("_binary_certificate_pem_crt_end");
extern const uint8_t private_pem_key_start[] asm("_binary_private_pem_key_start");
extern const uint8_t private_pem_key_end[] asm("_binary_private_pem_key_end");
extern const uint8_t s3_root_cert_start[] asm("_binary_s3_root_cert_pem_start");
extern const uint8_t s3_root_cert_end[] asm("_binary_s3_root_cert_pem_end");

/**
 * @brief Default MQTT HOST URL is pulled from the aws_iot_config.h
 */
char HostAddress[255] = AWS_IOT_MQTT_HOST;

/**
 * @brief Default MQTT port is pulled from the aws_iot_config.h
 */
uint32_t port = AWS_IOT_MQTT_PORT;

/* AWS Client Information */
static AWS_IoT_Client client;
static IoT_Client_Init_Params mqttInitParams;
static IoT_Client_Connect_Params connectParams;
static IoT_Publish_Message_Params paramsQOS0;

/* AWS test variables */
char testPayload[32];
const char *TEST_TOPIC = "test_topic/esp32";
const uint8_t TEST_TOPIC_LEN = strlen("test_topic/esp32");
int32_t testVar = 0;

/* AWS Topic and Storage Buffers */
static char getRejectedSubBuf[128];
static char getAcceptedSubBuf[128];
static char getJobAcceptedSubBuf[128];
static char getJobUpdateSubBuf[128];
static char tempJobBuf[128];
static char tempStringBuf[256];
//static char currentJobIdBuf[64];

/* Job storage */
static jobDefinition_t currentJob;

/* Jsmn JSON Parser */
static jsmn_parser jParser;
static jsmntok_t jTokens[128];

/* Private Function Prototypes
 * -------------------------------------------------------------------------- */

static bool extractJsonTokenAsString(const jsmntok_t *pToken, const char *source, char *dest, uint16_t destLen);
static void taskFatalError(void);
static esp_err_t event_handler(void *ctx, system_event_t *event);
static bool wifiProvisionedCheck(void);
static void initialiseWifi(void);
static bool diagnostic(void);
static void bootValidityCheck(void);
static esp_err_t nvsBootCheck(void);
static void otaBootCheck(void);

void disconnectCallbackHandler(AWS_IoT_Client *pClient, void *data);

void awsGetRejectedCallbackHandler(AWS_IoT_Client *pClient, char *topicName, uint16_t topicNameLen,
                                   IoT_Publish_Message_Params *params, void *pData);
void awsGetAcceptedCallbackHandler(AWS_IoT_Client *pClient, char *topicName, uint16_t topicNameLen,
                                   IoT_Publish_Message_Params *params, void *pData);
void awsJobGetAcceptedCallbackHandler(AWS_IoT_Client *pClient, char *topicName, uint16_t topicNameLen,
                                      IoT_Publish_Message_Params *params, void *pData);
void awsGetJobUpdateCallbackHandler(AWS_IoT_Client *pClient, char *topicName, uint16_t topicNameLen,
                                    IoT_Publish_Message_Params *params, void *pData);
void awsProcessJob(void);

static void connectToAWS(void);

static void unsubAndDisconnnectAWS(void);


/* -------------------------------------------------------------------------- */

static bool extractJsonTokenAsString(const jsmntok_t *pToken, const char *source, char *dest, uint16_t destLen)
{
	uint16_t sourceLen = pToken->end - pToken->start;

	/* Don't copy if no room in dest buffer for whole string + null terminator */
	if (sourceLen + 1 > destLen)
	{
		return false;
	}
	else
	{
		/* Copy from start of token in buffer over to destination*/
		memcpy(dest, &source[pToken->start], sourceLen);
		/* Append null terminator */
		dest[sourceLen] = '\0';
	}

	return true;
}


static void taskFatalError(void)
{
	while (1)
	{
		ESP_LOGE(TAG, "Fatal error in task, no return");
		vTaskDelay(10000 / portTICK_PERIOD_MS);
	}

    (void)vTaskDelete(NULL);
}


static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch(event->event_id) {
    case SYSTEM_EVENT_STA_START:
        esp_wifi_connect();
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
        xEventGroupSetBits(taskEventGroup, WIFI_CONNECTED_BIT);
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        /* This is a workaround as ESP32 WiFi libs don't currently
           auto-reassociate. */
        esp_wifi_connect();
        xEventGroupClearBits(taskEventGroup, WIFI_CONNECTED_BIT);
        break;
    default:
        break;
    }
    return ESP_OK;
}


static bool wifiProvisionedCheck(void)
{
	bool ret = true;



	return ret;
}


static void initialiseWifi(void)
{
    tcpip_adapter_init();
    ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_WIFI_SSID,
            .password = EXAMPLE_WIFI_PASS,
        },
    };
    ESP_LOGI(TAG, "Setting WiFi configuration SSID %s...", wifi_config.sta.ssid);
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK( esp_wifi_start() );
}


static bool diagnostic(void)
{
    /* This needs implementing
     * - Need to come up with test to check if app is valid
     * - Current implementation always passes
     */

    return true;
}


static void bootValidityCheck(void)
{
	const esp_partition_t *running = esp_ota_get_running_partition();
	esp_ota_img_states_t ota_state;
	if (esp_ota_get_state_partition(running, &ota_state) == ESP_OK)
	{
		if (ota_state == ESP_OTA_IMG_PENDING_VERIFY)
		{
			/* Run the diagnostic function */
			bool diagnostic_ok = diagnostic();
			if (diagnostic_ok)
			{
				ESP_LOGI(TAG, "Diagnostics completed successfully");
				esp_ota_mark_app_valid_cancel_rollback();
				// Notify AWS that job completed successfully
			}
			else
			{
				ESP_LOGE(TAG, "Diagnostics failed, rolling back to previous version");
				esp_ota_mark_app_invalid_rollback_and_reboot();
			}
		}
	}
}


static esp_err_t nvsBootCheck(void)
{
	/* Initialise NVS */
	esp_err_t err = nvs_flash_init();
	if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		/* OTA app partition table has a smaller NVS partition size than the non-OTA
		 * partition table. This size mismatch may cause NVS initialisation to fail.
		 * If this happens, we erase NVS partition and initialise NVS again. */
		ESP_ERROR_CHECK(nvs_flash_erase());
		err = nvs_flash_init();
	}
	return err;
}


static void otaBootCheck(void)
{
	const esp_partition_t *configured = esp_ota_get_boot_partition();
	const esp_partition_t *running = esp_ota_get_running_partition();

	if (configured != running)
	{
		ESP_LOGW(TAG, "Configured OTA boot partition at offset 0x%08x, but running from offset 0x%08x",
				 configured->address, running->address);
		ESP_LOGW(TAG, "This can happen if either the OTA boot data or preferred boot image becomes corrupted.");
	}

	ESP_LOGI(TAG, "Running partition type %d subtype %d (offset 0x%08x)",
			 running->type, running->subtype, running->address);
}


void disconnectCallbackHandler(AWS_IoT_Client *pClient, void *data)
{
    ESP_LOGW(TAG, "MQTT Disconnect");
    IoT_Error_t rc = FAILURE;

    if (NULL == pClient)
    {
    	ESP_LOGE(TAG, "MQTT Disconnect, AWS client NULL");
        return;
    }

    if (aws_iot_is_autoreconnect_enabled(pClient))
    {
        ESP_LOGI(TAG, "Auto Reconnect is enabled, Reconnecting attempt will start now");
    }
    else
    {
        ESP_LOGW(TAG, "Auto Reconnect not enabled. Starting manual reconnect...");
        rc = aws_iot_mqtt_attempt_reconnect(pClient);
        if (NETWORK_RECONNECTED == rc)
        {
            ESP_LOGW(TAG, "Manual Reconnect Successful");
        }
        else
        {
            ESP_LOGW(TAG, "Manual Reconnect Failed - %d", rc);
        }
    }
}


void awsGetRejectedCallbackHandler(AWS_IoT_Client *pClient, char *topicName, uint16_t topicNameLen,
                                   IoT_Publish_Message_Params *params, void *pData)
{
	ESP_LOGW(TAG, "\n Rejected: %.*s\t%.*s\n", topicNameLen, topicName, (int) params->payloadLen, (char *)params->payload);

	/* Check if a message has been received with any data inside */
	if ((params->payloadLen) <= 0)
	{
		ESP_LOGW(TAG, "Subscription Callback: Message payload has length of 0");
	}
	else
	{

	}
}


void awsGetAcceptedCallbackHandler(AWS_IoT_Client *pClient, char *topicName, uint16_t topicNameLen,
                                   IoT_Publish_Message_Params *params, void *pData)
{
	/* ESP_LOGI(TAG, "\n Accepted: %.*s\t%.*s\n", topicNameLen, topicName, (int) params->payloadLen, (char *)params->payload); */
	bool ret = false;

	/* Check if a message has been received with any data inside */
	if ((params->payloadLen) <= 0)
	{
		ESP_LOGW(TAG, "Subscription Callback: Message payload has length of 0");
	}
	else
	{
		/* Initialise jsmn parser and check it is valid */
		jsmn_init(&jParser);
		int r = jsmn_parse(&jParser, params->payload, params->payloadLen, jTokens, COUNT_OF(jTokens));

		/* Check if parsed json is valid */
		if (r < 0)
		{
			ESP_LOGE(TAG, "Job Accepted Callback: Failed to parse JSON: %d", r);
			taskFatalError();
		}

		if ((r < 1) || (jTokens[0].type != JSMN_OBJECT))
		{
			ESP_LOGE(TAG, "Job Accepted Callback: Expected object");
			taskFatalError();
		}

		/* Search through JSON for in progress jobs, as these need dealing with first */
		jsmntok_t *inProgressToken = findToken("inProgressJobs", params->payload, &jTokens[0]);
		if (inProgressToken != NULL)
		{
			if (inProgressToken->size == 0)
			{
				ESP_LOGI(TAG, "No in progress jobs");
			}
			else
			{
			    xEventGroupSetBits(taskEventGroup, JOB_IN_PROGRESS_BIT);

				ret = extractJsonTokenAsString(inProgressToken, (char *)params->payload, &tempStringBuf[0], COUNT_OF(tempStringBuf));
				if (!ret)
				{
					ESP_LOGE(TAG, "JSON token extraction fail, buffer overflow");
					taskFatalError();
				}
				ESP_LOGI(TAG, "Size: %d, In progress jobs: %s", inProgressToken->size, tempStringBuf);
			}
		}
		else
		{
			ESP_LOGE(TAG, "Token not found");
		}

		/* Search through JSON for queued jobs */
		jsmntok_t *queuedToken = findToken("queuedJobs", params->payload, &jTokens[0]);
		if (queuedToken != NULL)
		{
			if (queuedToken->size == 0)
			{
				ESP_LOGI(TAG, "No queued jobs");
			}
			else
			{
				ret = extractJsonTokenAsString(queuedToken, (char *)params->payload, &tempStringBuf[0], COUNT_OF(tempStringBuf));
				if (!ret)
				{
					ESP_LOGE(TAG, "JSON token extraction fail, buffer overflow");
					taskFatalError();
				}
				ESP_LOGI(TAG, "Size: %d, Queued jobs: %s", queuedToken->size, tempStringBuf);

				xEventGroupSetBits(taskEventGroup, JOB_IN_PROGRESS_BIT);

				/* Currently work through buffer from first job object */

				/* Since jobs are stored as objects within an array, search forwards to find next object(job) */
				jsmntok_t *jobToken = queuedToken;

				while (jobToken->type != JSMN_OBJECT)
				{
					jobToken++;
				}

				/* Job found */
				ret = extractJsonTokenAsString(jobToken, (char *)params->payload, &tempStringBuf[0], COUNT_OF(tempStringBuf));
				if (!ret)
				{
					ESP_LOGE(TAG, "JSON token extraction fail, buffer overflow");
					taskFatalError();
				}
				ESP_LOGI(TAG, "Size: %d, Job: %s", jobToken->size, tempStringBuf);

				jsmntok_t *jobIdToken = findToken("jobId", params->payload, jobToken);
				ret = extractJsonTokenAsString(jobIdToken, (char *)params->payload, &currentJob.jobId[0], COUNT_OF(currentJob.jobId));
				if (!ret)
				{
					ESP_LOGE(TAG, "JSON token extraction fail, buffer overflow");
					taskFatalError();
				}
				ESP_LOGI(TAG, "Job Id: %s", currentJob.jobId);

				/* Trigger job execution by asking AWS for job document.
				 * - Job execution takes place in awsJobGetAcceptedCallback */
				AwsIotDescribeJobExecutionRequest describeRequest;
				describeRequest.executionNumber = 0;
				describeRequest.includeJobDocument = true;
				describeRequest.clientToken = NULL;
				IoT_Error_t rc = aws_iot_jobs_describe(pClient, QOS0, THING_NAME, currentJob.jobId, &describeRequest,
													   tempJobBuf, COUNT_OF(tempJobBuf), NULL, 0);
				if (SUCCESS != rc)
				{
					ESP_LOGE(TAG, "Unable to publish job description request: %d", rc);
					taskFatalError();
				}
			}
		}
		else
		{
			ESP_LOGE(TAG, "Token not found");
		}
	}
}


void awsJobGetAcceptedCallbackHandler(AWS_IoT_Client *pClient, char *topicName, uint16_t topicNameLen,
                                   IoT_Publish_Message_Params *params, void *pData)
{
	/* ESP_LOGI(TAG, "\n Job Accepted: %.*s\t%.*s\n", topicNameLen, topicName, (int) params->payloadLen, (char *)params->payload); */
	bool ret  = false;

	/* Check if a message has been received with any data inside */
	if ((params->payloadLen) <= 0)
	{
		ESP_LOGW(TAG, "Subscription Callback: Message payload has length of 0");
	}
	else
	{
		/* Initialise jsmn parser and check it is valid */
		jsmn_init(&jParser);
		int r = jsmn_parse(&jParser, params->payload, params->payloadLen, jTokens, COUNT_OF(jTokens));

		/* Check if parsed json is valid */
		if (r < 0)
		{
			ESP_LOGE(TAG, "Job Accepted Callback: Failed to parse JSON: %d", r);
			taskFatalError();
		}

		if ((r < 1) || (jTokens[0].type != JSMN_OBJECT))
		{
			ESP_LOGE(TAG, "Job Accepted Callback: Expected object");
			taskFatalError();
		}

		/* First must find execution document */
		jsmntok_t *execDocToken = findToken("execution", params->payload, &jTokens[0]);
		ret = extractJsonTokenAsString(execDocToken, (char *)params->payload, &tempStringBuf[0], COUNT_OF(tempStringBuf));
		if (!ret)
		{
			ESP_LOGE(TAG, "JSON token extraction fail, buffer overflow");
			taskFatalError();
		}
		ESP_LOGI(TAG, "Exec Doc: %s", tempStringBuf);

		/* Then find job document inside execution document and store in current job*/
		jsmntok_t *jobDocToken = findToken("jobDocument", params->payload, execDocToken);
		ret = extractJsonTokenAsString(jobDocToken, (char *)params->payload, &currentJob.jobDoc[0], COUNT_OF(currentJob.jobDoc));
		if (!ret)
		{
			ESP_LOGE(TAG, "JSON token extraction fail, buffer overflow");
			taskFatalError();
		}
		ESP_LOGI(TAG, "Job Doc: %s", currentJob.jobDoc);

		/* Since a job has been found, set JOB_READY_BIT */
        xEventGroupSetBits(taskEventGroup, JOB_READY_BIT);
	}
}


void awsGetJobUpdateCallbackHandler(AWS_IoT_Client *pClient, char *topicName, uint16_t topicNameLen,
                                    IoT_Publish_Message_Params *params, void *pData)
{
    ESP_LOGI(TAG, "\n Job Update Callback: %.*s\t%.*s\n", topicNameLen, topicName, (int) params->payloadLen, (char *)params->payload);
}


void awsProcessJob(void)
{
    bool ret = false;

    xEventGroupSetBits(taskEventGroup, JOB_IN_PROGRESS_BIT);

    jsmn_init(&jParser);
    int r = jsmn_parse(&jParser, currentJob.jobDoc, COUNT_OF(currentJob.jobDoc), jTokens, COUNT_OF(jTokens));

    /* Check if parsed json is valid */
    if (r < 0)
    {
        ESP_LOGE(TAG, "Process Job: Failed to parse JSON: %d", r);
        taskFatalError();
    }

    if ((r < 1) || (jTokens[0].type != JSMN_OBJECT))
    {
        ESP_LOGE(TAG, "Process Job: Expected object");
        taskFatalError();
    }

    /* Then find job type */
    jsmntok_t *jobTypeToken = findToken("operation", currentJob.jobDoc, &jTokens[0]);
    ret = extractJsonTokenAsString(jobTypeToken, (char *)currentJob.jobDoc, &tempStringBuf[0], COUNT_OF(tempStringBuf));
    if (!ret)
    {
        ESP_LOGE(TAG, "JSON token extraction fail, buffer overflow");
        taskFatalError();
    }
    ESP_LOGI(TAG, "Job Type: %s", tempStringBuf);

    if (strcmp(tempStringBuf, "ota") == 0)
    {
        /* For OTA we expect url contained in job doc so search for it*/
        jsmntok_t *urlToken = findToken("url", currentJob.jobDoc, &jTokens[0]);
        ret = extractJsonTokenAsString(urlToken, (char *)currentJob.jobDoc, &tempStringBuf[0], COUNT_OF(tempStringBuf));
        if (!ret)
        {
            ESP_LOGE(TAG, "JSON token extraction fail, buffer overflow");
            taskFatalError();
        }
        ESP_LOGI(TAG, "url: %s", tempStringBuf);

        ma_ota_err_t otaErr = httpsOtaUpdate(&tempStringBuf[0], s3_root_cert_start);
        if (otaErr != MA_OTA_SUCCESS)
        {
            ESP_LOGE(TAG, "HTTPS OTA update failed: %d", otaErr);
            taskFatalError();
            xEventGroupSetBits(taskEventGroup, JOB_FAILED_BIT);
        }
        else
        {
            ESP_LOGI(TAG, "HTTPS OTA update successful, request restart");
            xEventGroupSetBits(taskEventGroup, RESTART_REQUESTED_BIT | JOB_COMPLETED_BIT);
        }
    }
    else if (strcmp(tempStringBuf, "awsMsg") == 0)
    {

    }
    else if (strcmp(tempStringBuf, "reboot") == 0)
    {

    }
    else
    {
        ESP_LOGE(TAG, "Job operation type not found: %s", tempStringBuf);
    }
}


static void connectToAWS(void)
{
	IoT_Error_t rc = FAILURE;

	mqttInitParams = iotClientInitParamsDefault;
	connectParams = iotClientConnectParamsDefault;

    ESP_LOGI(TAG, "AWS IoT SDK Version %d.%d.%d-%s", VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH, VERSION_TAG);

    mqttInitParams.enableAutoReconnect = false; // We enable this later below
    mqttInitParams.pHostURL = HostAddress;
    mqttInitParams.port = port;

    mqttInitParams.pRootCALocation = (const char *)aws_root_ca_pem_start;
    mqttInitParams.pDeviceCertLocation = (const char *)certificate_pem_crt_start;
    mqttInitParams.pDevicePrivateKeyLocation = (const char *)private_pem_key_start;

    mqttInitParams.mqttCommandTimeout_ms = 30000;
    mqttInitParams.tlsHandshakeTimeout_ms = 5000;
    mqttInitParams.isSSLHostnameVerify = true;
    mqttInitParams.disconnectHandler = disconnectCallbackHandler;
    mqttInitParams.disconnectHandlerData = NULL;

    rc = aws_iot_mqtt_init(&client, &mqttInitParams);
    if (SUCCESS != rc)
    {
        ESP_LOGE(TAG, "aws_iot_mqtt_init returned error : %d ", rc);
        taskFatalError();
    }

    /* Reasonably long keep alive interval to deal with MQTT dead time during OTA update */
    connectParams.keepAliveIntervalInSec = 30;
    connectParams.isCleanSession = true;
    connectParams.MQTTVersion = MQTT_3_1_1;
    /* Client ID is set in the menuconfig of the example */
    connectParams.pClientID = CONFIG_AWS_EXAMPLE_CLIENT_ID;
    connectParams.clientIDLen = (uint16_t) strlen(CONFIG_AWS_EXAMPLE_CLIENT_ID);
    connectParams.isWillMsgPresent = false;

    ESP_LOGI(TAG, "Connecting to AWS...");
	do
	{
		rc = aws_iot_mqtt_connect(&client, &connectParams);
		if (SUCCESS != rc)
		{
			ESP_LOGE(TAG, "Error(%d) connecting to %s:%d", rc, mqttInitParams.pHostURL, mqttInitParams.port);
			vTaskDelay(1000 / portTICK_PERIOD_MS);
		}
	} while(SUCCESS != rc);

	/*
	 * Enable Auto Reconnect functionality. Minimum and Maximum time of Exponential backoff are set in aws_iot_config.h
	 *  #AWS_IOT_MQTT_MIN_RECONNECT_WAIT_INTERVAL
	 *  #AWS_IOT_MQTT_MAX_RECONNECT_WAIT_INTERVAL
	 */
	rc = aws_iot_mqtt_autoreconnect_set_status(&client, true);
	if (SUCCESS != rc)
	{
		ESP_LOGE(TAG, "Unable to set Auto Reconnect to true - %d", rc);
		taskFatalError();
	}

    paramsQOS0.qos = QOS0;
    paramsQOS0.payload = (void *) testPayload;
    paramsQOS0.isRetained = 0;

    /* Create and subscribe to job topics */
    snprintf(getRejectedSubBuf, COUNT_OF(getRejectedSubBuf), "$aws/things/%s/jobs/get/rejected", THING_NAME);
    rc = aws_iot_mqtt_subscribe(&client, getRejectedSubBuf, COUNT_OF(getRejectedSubBuf), QOS0,
    							awsGetRejectedCallbackHandler, NULL);

    snprintf(getAcceptedSubBuf, COUNT_OF(getAcceptedSubBuf), "$aws/things/%s/jobs/get/accepted", THING_NAME);
    rc = aws_iot_mqtt_subscribe(&client, getAcceptedSubBuf, COUNT_OF(getAcceptedSubBuf), QOS0,
    							awsGetAcceptedCallbackHandler, NULL);

    snprintf(getJobAcceptedSubBuf, COUNT_OF(getJobAcceptedSubBuf), "$aws/things/%s/jobs/+/get/accepted", THING_NAME);
    rc = aws_iot_mqtt_subscribe(&client, getJobAcceptedSubBuf, COUNT_OF(getJobAcceptedSubBuf), QOS0,
        						awsJobGetAcceptedCallbackHandler, NULL);

    if (SUCCESS != rc)
	{
		ESP_LOGE(TAG, "Unable to subscribe to AWS Jobs service: %d", rc);
		taskFatalError();
	}

    /* Loop for AWS processing stage */
    while ((NETWORK_ATTEMPTING_RECONNECT == rc || NETWORK_RECONNECTED == rc || SUCCESS == rc))
    {
    	ESP_LOGI(TAG, "In AWS processing loop\n\n");

        // Yield pauses the current thread, allowing MQTT send/receive to occur
        rc = aws_iot_mqtt_yield(&client, 100);

        if(NETWORK_ATTEMPTING_RECONNECT == rc)
        {
            // If the client is attempting to reconnect then skip the rest of the loop.
            vTaskDelay(100 / portTICK_PERIOD_MS);
            continue;
        }

        /* If a restart is requested, unsub and then restart */
        if ((xEventGroupGetBits(taskEventGroup) & RESTART_REQUESTED_BIT) != 0)
        {
            ESP_LOGI(TAG, "RESTART_REQUESTED_BIT processing");
            unsubAndDisconnnectAWS();
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            esp_restart();
        }

        /* Only query for more jobs if not currently processing a job */
        if ((xEventGroupGetBits(taskEventGroup) & JOB_IN_PROGRESS_BIT) == 0)
        {
            ESP_LOGI(TAG, "AWS_QUERY processing");
            rc = aws_iot_jobs_send_query(&client, QOS0, THING_NAME, NULL, NULL, tempJobBuf, COUNT_OF(tempJobBuf),
                                         NULL, 0, JOB_GET_PENDING_TOPIC);
        }

        /* We can process a job locally if a job is ready from AWS */
        if ((xEventGroupGetBits(taskEventGroup) & JOB_READY_BIT) != 0)
        {
            ESP_LOGI(TAG, "JOB_READY_BIT processing");

            /* Create and subscribe to job topics */
            snprintf(getJobUpdateSubBuf, COUNT_OF(getJobUpdateSubBuf), "$aws/things/%s/jobs/%s/update/#", THING_NAME, currentJob.jobId);
            rc = aws_iot_mqtt_subscribe(&client, getJobUpdateSubBuf, COUNT_OF(getJobUpdateSubBuf), QOS0,
                                        awsGetJobUpdateCallbackHandler, NULL);
            if (rc != SUCCESS)
            {
                ESP_LOGE(TAG, "AWS Job Update sub failed: %d", rc);
                taskFatalError();
            }

            AwsIotJobExecutionUpdateRequest updateRequest;
            updateRequest.clientToken = NULL;
            updateRequest.includeJobDocument = false;
            updateRequest.includeJobExecutionState = false;
            updateRequest.status = JOB_EXECUTION_IN_PROGRESS;
            rc = aws_iot_jobs_send_update(&client, QOS0, THING_NAME, currentJob.jobId, &updateRequest, tempJobBuf, COUNT_OF(tempJobBuf), tempStringBuf, COUNT_OF(tempStringBuf));
            if (rc != SUCCESS)
            {
                ESP_LOGE(TAG, "AWS Job Update notification failed: %d", rc);
                taskFatalError();
            }
            awsProcessJob();
        }

        /* We can notify AWS as job completed once verified complete */
        if ((xEventGroupGetBits(taskEventGroup) & JOB_COMPLETED_BIT) != 0)
        {
            ESP_LOGI(TAG, "JOB_COMPLETED_BIT processing");

            /* Notify AWS that current job has completed successfully */
            AwsIotJobExecutionUpdateRequest updateRequest;
            updateRequest.clientToken = NULL;
            updateRequest.includeJobDocument = false;
            updateRequest.includeJobExecutionState = false;
            updateRequest.status = JOB_EXECUTION_SUCCEEDED;
            rc = aws_iot_jobs_send_update(&client, QOS0, THING_NAME, currentJob.jobId, &updateRequest, tempJobBuf, COUNT_OF(tempJobBuf), tempStringBuf, COUNT_OF(tempStringBuf));
            if (rc != SUCCESS)
            {
                ESP_LOGE(TAG, "AWS Job Update notification failed: %d", rc);
                taskFatalError();
            }

            /* Reset all in job in completion flags to continue processing jobs */
            xEventGroupClearBits(taskEventGroup, JOB_IN_PROGRESS_BIT | JOB_COMPLETED_BIT | JOB_READY_BIT);
        }

        /* Notify AWS if current job failed */
        if ((xEventGroupGetBits(taskEventGroup) & JOB_FAILED_BIT) != 0)
        {
            ESP_LOGW(TAG, "JOB_FAILED_BIT processing");

            AwsIotJobExecutionUpdateRequest updateRequest;
            updateRequest.clientToken = NULL;
            updateRequest.includeJobDocument = false;
            updateRequest.includeJobExecutionState = false;
            updateRequest.status = JOB_EXECUTION_FAILED;
            rc = aws_iot_jobs_send_update(&client, QOS0, THING_NAME, currentJob.jobId, &updateRequest, tempJobBuf, COUNT_OF(tempJobBuf), tempStringBuf, COUNT_OF(tempStringBuf));
            if (rc != SUCCESS)
            {
                ESP_LOGE(TAG, "AWS Job Update notification failed: %d", rc);
                taskFatalError();
            }

            /* Reset all in job in completion flags to continue processing jobs */
            xEventGroupClearBits(taskEventGroup, JOB_IN_PROGRESS_BIT | JOB_COMPLETED_BIT | JOB_READY_BIT);
        }

        ESP_LOGI(TAG, "Stack remaining for task '%s' is %d bytes", pcTaskGetTaskName(NULL), uxTaskGetStackHighWaterMark(NULL));
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }

    unsubAndDisconnnectAWS();
}


static void unsubAndDisconnnectAWS(void)
{
    IoT_Error_t rc = FAILURE;

    rc = aws_iot_mqtt_yield(&client, 100);

    /* Unsubscribe from all AWS services */
    rc = aws_iot_jobs_unsubscribe_from_job_messages(&client, getRejectedSubBuf);
    rc = aws_iot_jobs_unsubscribe_from_job_messages(&client, getAcceptedSubBuf);
    rc = aws_iot_jobs_unsubscribe_from_job_messages(&client, getJobAcceptedSubBuf);

    rc = aws_iot_mqtt_yield(&client, 100);

    if (rc != SUCCESS)
    {
        ESP_LOGE(TAG, "Unable to unsubscribe from AWS Jobs service: %d", rc);
        taskFatalError();
    }
    else
    {
        ESP_LOGI(TAG, "Unsubscribed from AWS jobs services");
    }

    /* Disconnect from AWS */
    rc = aws_iot_mqtt_disconnect(&client);
    if (rc != SUCCESS)
    {
        ESP_LOGE(TAG, "Unable to disconnect from AWS: %d", rc);
        taskFatalError();
    }
    else
    {
        ESP_LOGI(TAG, "Disconnected from AWS");
    }
}


void job_agent_task(void *param)
{
    taskEventGroup = xEventGroupCreate();

	bootValidityCheck();
	ESP_ERROR_CHECK(nvsBootCheck());
	otaBootCheck();

	initialiseWifi();

	while (1)
	{
		/* Check if WiFi provisioned */
		if (wifiProvisionedCheck() == 1)
		{
			ESP_LOGI(TAG, "WiFi provisioned");
		}
		else
		{
			ESP_LOGW(TAG, "WiFi not provisioned, waiting on credentials");
			vTaskDelay(2000 / portTICK_PERIOD_MS);
			continue;
		}

		/* Check if WiFi connected */
		EventBits_t ret_bit = xEventGroupWaitBits(taskEventGroup, WIFI_CONNECTED_BIT, false, true, 50 / portTICK_PERIOD_MS);
		if ((ret_bit & WIFI_CONNECTED_BIT) != 0)
		{
			ESP_LOGI(TAG, "WiFi connected");
		}
		else
		{
			ESP_LOGW(TAG, "WiFi not connected");
			vTaskDelay(2000 / portTICK_PERIOD_MS);
			continue;
		}

		/* Connect to AWS and process jobs*/
		connectToAWS();

		ESP_LOGE(TAG, "An error occurred in the main loop.");
		taskFatalError();
	}
}
