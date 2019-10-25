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


/* -------------------------------------------------------------------------- */


static const char *TAG = "ma_JobOTA";


/* Private Function Prototypes
 * -------------------------------------------------------------------------- */
static esp_err_t validate_image_header(esp_app_desc_t *new_app_info);


/* Private Function Definitions
 * -------------------------------------------------------------------------- */
static esp_err_t validate_image_header(esp_app_desc_t *new_app_info)
{

}


/* Function Definitions
 * -------------------------------------------------------------------------- */
esp_err_t httpsOtaUpdate(const char* url, const char* serverCertPemStart)
{
    ESP_LOGI(TAG, "Starting OTA update process");

    //esp_err_t errFinish = ESP_OK;

    esp_http_client_config_t config;
    config.url = url;
    config.cert_pem = serverCertPemStart;

    esp_https_ota_config_t otaConfig;
    otaConfig.http_config = &config;

    esp_https_ota_handle_t otaHandle = NULL;
    esp_err_t err = esp_https_ota_begin(&otaConfig, &otaHandle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "ESP HTTPS OTA Begin failed");
        return err;
    }

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


    return err;
}


