/**
 * @file example_driver.c
 *
 * @brief File to drive the ma_jobAgent component.
 *
 * The contents within the app_main func should be moved to any program that
 * makes use of the ma_jobAgent component.
 *
 * @author Jack Cripps // jackc@monitoraudio.com
 * @date 11/09/2019
 *
 */

#include "ma_jobAgent.h"

#include <stdio.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_event.h"

#include "nvs.h"
#include "nvs_flash.h"

/* -------------------------------------------------------------------------- */

static const char *TAG = "example_driver";

/* -------------------------------------------------------------------------- */

void app_main()
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

    /* Initialise jobAgent event group */
    jobAgentEventGroup = xEventGroupCreate();

    /* Minimum stack size for this task is used here, do not reduce */
    xTaskCreate(&job_agent_task, "job_agent_task", 1024 * 6, NULL, 5, NULL);

    while (1)
    {
        if ((xEventGroupGetBits(jobAgentEventGroup) & JOB_OTA_RESTART_BIT) != 0)
        {
            ESP_LOGW(TAG, "Job Agent requested a restart, restarting now");
            esp_restart();
        }

        if ((xEventGroupGetBits(jobAgentEventGroup) & JOB_OTA_REQUEST_BIT) != 0)
        {
            ESP_LOGW(TAG, "Job Agent has an OTA update request, approve by setting reply bit");
            xEventGroupSetBits(jobAgentEventGroup, JOB_OTA_REPLY_BIT);
        }

        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}
