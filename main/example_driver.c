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
#include <stdio.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"

#include "ma_jobAgent.h"

/* -------------------------------------------------------------------------- */

static const char *TAG = "example_driver";

/* -------------------------------------------------------------------------- */

void app_main()
{
    xTaskCreate(&job_agent_task, "job_agent_task", 1024 * 8, NULL, 5, NULL);
}
