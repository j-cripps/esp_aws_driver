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
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "driver/gpio.h"

#include "jsmn.h"

#include "nvs.h"
#include "nvs_flash.h"

#include "esp_ota_ops.h"
#include "esp_flash_partitions.h"
#include "esp_partition.h"

#include "aws_iot_config.h"
#include "aws_iot_log.h"
#include "aws_iot_version.h"
#include "aws_iot_mqtt_client_interface.h"
#include "aws_iot_jobs_interface.h"


#define COUNT_OF(x) ((sizeof(x)/sizeof(0[x])) / ((size_t)(!(sizeof(x) % sizeof(0[x])))))

/* -------------------------------------------------------------------------- */


#define EXAMPLE_WIFI_SSID "Monitor Audio WiFi"
#define EXAMPLE_WIFI_PASS "des1gnf0rs0und"


/* -------------------------------------------------------------------------- */


static const char *TAG = "ma_JobAgent";

static const char *THING_NAME = "ESP32TestThing";

/* FreeRTOS event group to signal when we are connected & ready to make a request */
static EventGroupHandle_t wifi_event_group;

/* The event group allows multiple bits for each event,
 * but we only care about one event - are we connected
 * to the AP with an IP?
 */
const int WIFI_CONNECTED_BIT = BIT0;

/* CA Root certificate, device ("Thing") certificate and device
 * ("Thing") key.
 */
extern const uint8_t aws_root_ca_pem_start[] asm("_binary_aws_root_ca_pem_start");
extern const uint8_t aws_root_ca_pem_end[] asm("_binary_aws_root_ca_pem_end");
extern const uint8_t certificate_pem_crt_start[] asm("_binary_certificate_pem_crt_start");
extern const uint8_t certificate_pem_crt_end[] asm("_binary_certificate_pem_crt_end");
extern const uint8_t private_pem_key_start[] asm("_binary_private_pem_key_start");
extern const uint8_t private_pem_key_end[] asm("_binary_private_pem_key_end");

/**
 * @brief Default MQTT HOST URL is pulled from the aws_iot_config.h
 */
char HostAddress[255] = AWS_IOT_MQTT_HOST;

/**
 * @brief Default MQTT port is pulled from the aws_iot_config.h
 */
uint32_t port = AWS_IOT_MQTT_PORT;

/* AWS Client Information */
AWS_IoT_Client client;
IoT_Client_Init_Params mqttInitParams;
IoT_Client_Connect_Params connectParams;
IoT_Publish_Message_Params paramsQOS0;

/* AWS test variables */
char testPayload[32];
const char *TEST_TOPIC = "test_topic/esp32";
const uint8_t TEST_TOPIC_LEN = strlen("test_topic/esp32");
int32_t testVar = 0;

/* AWS Topic Buffers */
char getRejectedSubBuf[128];
char getAcceptedSubBuf[128];
char getJobAcceptedSubBuf[128];
char getNotifyJobSubBuf[128];
char tempJobBuf[128];

/* Jsmn JSON Parser */
jsmn_parser jParser;
jsmntok_t jTokens[128];

/* Private Function Prototypes
 * -------------------------------------------------------------------------- */


static void __attribute__((noreturn)) task_fatal_error(void);
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
void awsProcessJob(AWS_IoT_Client *pClient, char *topicName, uint16_t topicNameLen,
                   IoT_Publish_Message_Params *params, void *pData);
static void connectToAWS(void);





/* -------------------------------------------------------------------------- */


static void __attribute__((noreturn)) task_fatal_error(void)
{
    ESP_LOGE(TAG, "Exiting task due to fatal error...");
    (void)vTaskDelete(NULL);
    // while(1) {};
}


static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch(event->event_id) {
    case SYSTEM_EVENT_STA_START:
        esp_wifi_connect();
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        /* This is a workaround as ESP32 WiFi libs don't currently
           auto-reassociate. */
        esp_wifi_connect();
        xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
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
    wifi_event_group = xEventGroupCreate();
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
    gpio_config_t io_conf;
    io_conf.intr_type    = GPIO_PIN_INTR_DISABLE;
    io_conf.mode         = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << CONFIG_EXAMPLE_GPIO_DIAGNOSTIC);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en   = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);

    ESP_LOGI(TAG, "Diagnostics (5 sec)...");
    vTaskDelay(5000 / portTICK_PERIOD_MS);

    bool diagnostic_is_ok = gpio_get_level(CONFIG_EXAMPLE_GPIO_DIAGNOSTIC);

    gpio_reset_pin(CONFIG_EXAMPLE_GPIO_DIAGNOSTIC);
    return 0;
}


static void bootValidityCheck(void)
{
	const esp_partition_t *running = esp_ota_get_running_partition();
	esp_ota_img_states_t ota_state;
	if (esp_ota_get_state_partition(running, &ota_state) == ESP_OK)
	{
		if (ota_state == ESP_OTA_IMG_PENDING_VERIFY)
		{
			// Run the diagnostic function
			bool diagnostic_ok = diagnostic();
			if (diagnostic_ok)
			{
				ESP_LOGI(TAG, "Diagnostics completed successfully");
				esp_ota_mark_app_valid_cancel_rollback();
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
	// Initialise NVS.
	esp_err_t err = nvs_flash_init();
	if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		// OTA app partition table has a smaller NVS partition size than the non-OTA
		// partition table. This size mismatch may cause NVS initialisation to fail.
		// If this happens, we erase NVS partition and initialise NVS again.
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
        if(NETWORK_RECONNECTED == rc)
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

	// Check if a message has been received with any data inside
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
	ESP_LOGI(TAG, "\n Accepted: %.*s\t%.*s\n", topicNameLen, topicName, (int) params->payloadLen, (char *)params->payload);

	// Check if a message has been received with any data inside
	if ((params->payloadLen) <= 0)
	{
		ESP_LOGW(TAG, "Subscription Callback: Message payload has length of 0");
	}
	else
	{
		jsmn_init(&jParser);
		int r = jsmn_parse(&jParser, params->payload, params->payloadLen, jTokens, COUNT_OF(jTokens));

		/* Check if parsed json is valid */
		if (r < 0)
		{
			ESP_LOGE(TAG, "Job Accepted Callback: Failed to parse JSON: %d", r);
			return;
		}

		if ((r < 1) || (jTokens[0].type != JSMN_OBJECT))
		{
			ESP_LOGE(TAG, "Job Accepted Callback: Expected object");
			return;
		}

		/* Search through JSON for in progress jobs, as these need dealing with first */
		jsmntok_t *inProgressToken = findToken("inProgressJobs", params->payload, &jTokens[0]);
		if (inProgressToken != NULL)
		{
			if (inProgressToken->end - inProgressToken->start < 3)
			{
				ESP_LOGI(TAG, "No in progress jobs");
			}
			else
			{
				ESP_LOGI(TAG, "In progress jobs present");
				strncpy(&tempJobBuf[0], &params->payload[inProgressToken->start], (inProgressToken->end - inProgressToken->start));
				tempJobBuf[inProgressToken->end - inProgressToken->start] = '\0';
				ESP_LOGI(TAG, "Found token: %s", tempJobBuf);
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
			if (queuedToken->end - queuedToken->start < 3)
			{
				ESP_LOGI(TAG, "No queued jobs");
			}
			else
			{
				ESP_LOGI(TAG, "Queued jobs present");
				strncpy(&tempJobBuf[0], &params->payload[queuedToken->start], (queuedToken->end - queuedToken->start));
				tempJobBuf[queuedToken->end - queuedToken->start] = '\0';
				ESP_LOGI(TAG, "Found token: %s", tempJobBuf);
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
	ESP_LOGI(TAG, "\n Job Accepted: %.*s\t%.*s\n", topicNameLen, topicName, (int) params->payloadLen, (char *)params->payload);

	// Check if a message has been received with any data inside
	if ((params->payloadLen) <= 0)
	{
		ESP_LOGW(TAG, "Subscription Callback: Message payload has length of 0");
	}
	else
	{

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

    mqttInitParams.mqttCommandTimeout_ms = 20000;
    mqttInitParams.tlsHandshakeTimeout_ms = 5000;
    mqttInitParams.isSSLHostnameVerify = true;
    mqttInitParams.disconnectHandler = disconnectCallbackHandler;
    mqttInitParams.disconnectHandlerData = NULL;

    rc = aws_iot_mqtt_init(&client, &mqttInitParams);
    if (SUCCESS != rc)
    {
        ESP_LOGE(TAG, "aws_iot_mqtt_init returned error : %d ", rc);
        abort();
    }

    connectParams.keepAliveIntervalInSec = 10;
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
		abort();
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
		abort();
	}

    /* Loop for AWS processing stage */
    while ((NETWORK_ATTEMPTING_RECONNECT == rc || NETWORK_RECONNECTED == rc || SUCCESS == rc))
    {
    	ESP_LOGI(TAG, "In AWS processing loop");

        // Yield pauses the current thread, allowing MQTT send/receive to occur
        rc = aws_iot_mqtt_yield(&client, 100);

        if(NETWORK_ATTEMPTING_RECONNECT == rc)
        {
            // If the client is attempting to reconnect then skip the rest of the loop.
            continue;
        }

//        testVar++;
//        sprintf(testPayload, "Hello %d", testVar);
//        paramsQOS0.payload = (void *)testPayload;
//        paramsQOS0.payloadLen = strlen(testPayload);
//        rc = aws_iot_mqtt_publish(&client, TEST_TOPIC, TEST_TOPIC_LEN, &paramsQOS0);
//        if (rc != SUCCESS)
//		{
//			ESP_LOGE(TAG, "Error subscribing to all jobs: %d", rc);
//			abort();
//		}

        rc = aws_iot_jobs_send_query(&client, QOS0, THING_NAME, NULL, NULL, tempJobBuf, COUNT_OF(tempJobBuf),
									 NULL, NULL, JOB_GET_PENDING_TOPIC);

        ESP_LOGI(TAG, "Stack remaining for task '%s' is %d bytes", pcTaskGetTaskName(NULL), uxTaskGetStackHighWaterMark(NULL));
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }

    /* Unsubscribe from all AWS services */
    rc = aws_iot_jobs_unsubscribe_from_job_messages(&client, getRejectedSubBuf);
    rc = aws_iot_jobs_unsubscribe_from_job_messages(&client, getAcceptedSubBuf);
    rc = aws_iot_jobs_unsubscribe_from_job_messages(&client, getJobAcceptedSubBuf);
    if (SUCCESS != rc)
	{
		ESP_LOGE(TAG, "Unable to unsubscribe from AWS Jobs service: %d", rc);
		abort();
	}
    else
    {
    	ESP_LOGI(TAG, "Unsubscribed from AWS jobs services");
    }

    /* Disconnect from AWS */
    rc = aws_iot_mqtt_disconnect(&client);
    if (SUCCESS != rc)
	{
		ESP_LOGE(TAG, "Unable to disconnect from AWS: %d", rc);
		abort();
	}
    else
    {
    	ESP_LOGI(TAG, "Disconnected from AWS");
    }
}


void job_agent_task(void *param)
{
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
		EventBits_t ret_bit = xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT, false, true, 50 / portTICK_PERIOD_MS);
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
		abort();
	}
}
