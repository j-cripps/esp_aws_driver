/**
 * @file ma_jobOTA.c
 *
 * @brief Contains functions relating to the OTA (over-the-air) update feature
 *
 * @note Based on the 'advanced_https_ota_example' in ESP-IDF
 *
 * @author Jack Cripps // jackc@monitoraudio.com
 * @date 25/10/2019
 */

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

#include "esp_ota_ops.h"
#include "esp_http_client.h"
#include "esp_https_ota.h"
#include "esp_flash_partitions.h"
#include "esp_partition.h"
#include "nvs.h"
#include "nvs_flash.h"


/* -------------------------------------------------------------------------- */


static const char *TAG = "ma_JobOTA";


/* Private Function Prototypes
 * -------------------------------------------------------------------------- */
static esp_err_t validate_image_header(esp_app_desc_t *new_app_info);


/* Private Function Definitions
 * -------------------------------------------------------------------------- */
static esp_err_t validate_image_header(esp_app_desc_t *new_app_info)
{
    if (new_app_info == NULL)
    {
        ESP_LOGE(TAG, "App info null in validation");
        return ESP_ERR_INVALID_ARG;
    }

    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_app_desc_t runningAppInfo;
    if (esp_ota_get_partition_description(running, &runningAppInfo))
    {
        ESP_LOGI(TAG, "Running firmware version: %s", runningAppInfo.version);
    }

    if (memcmp(new_app_info->version, runningAppInfo.version, sizeof(new_app_info->version)) == 0)
    {
        ESP_LOGW(TAG, "Current running version is same as new version, update failed");
        return ESP_FAIL;
    }

    return ESP_OK;
}


/* Function Definitions
 * -------------------------------------------------------------------------- */
esp_err_t httpsOtaUpdate(const char* url, const uint8_t* serverCertPemStart, const uint8_t* clientKeyPemStart)
{
    ESP_LOGI(TAG, "Starting OTA update process");

    esp_err_t errFinish = ESP_OK;

    esp_http_client_config_t config;
    config.url = url;
    config.cert_pem = (char *)serverCertPemStart;
    //config.client_key_pem = (char *)clientKeyPemStart;

    esp_https_ota_config_t otaConfig;
    otaConfig.http_config = &config;

    esp_https_ota_handle_t otaHandle = NULL;
    esp_err_t err = esp_https_ota_begin(&otaConfig, &otaHandle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "ESP HTTPS OTA Begin failed");
        return err;
    }

    ESP_LOGW(TAG, "Reached Here");

    esp_app_desc_t appDesc;
    err = esp_https_ota_get_img_desc(otaHandle, &appDesc);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_https_ota_read_img_desc failed");
        return err;
    }

    err = validate_image_header(&appDesc);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "ota image header verification failed");
        return err;
    }

    /* Main body of OTA update */
    while (1)
    {
        err = esp_https_ota_perform(otaHandle);
        if (err != ESP_ERR_HTTPS_OTA_IN_PROGRESS)
        {
            break;
        }

        /* esp_https_ota_perform returns after every read operation which gives user the ability to
         * monitor the status of OTA upgrade by calling esp_https_ota_get_image_len_read, which gives length of image
         * data read so far. */
        ESP_LOGD(TAG, "Image bytes read %d", esp_https_ota_get_image_len_read(otaHandle));
    }

    errFinish = esp_https_ota_finish(otaHandle);
    if ((err == ESP_OK) && (errFinish == ESP_OK))
    {
        ESP_LOGI(TAG, "OTA Upgrade Successful, marked for reboot");

        /* Temp test, this should occur somewhere else, at least in main task func so more control */
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        esp_restart();
    }
    else
    {
        ESP_LOGE(TAG, "OTA Upgrade failed: %d", errFinish);
    }

    return err;
}


