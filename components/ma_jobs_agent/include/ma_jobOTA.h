/**
 * @file ma_jobOTA.h
 *
 * @brief Contains functions relating to the OTA (over-the-air) update feature
 *
 * @author Jack Cripps // jackc@monitoraudio.com
 * @date 25/10/2019
 */

#ifndef __ma_jobOTA__
#define __ma_jobOTA__


/* Function Prototypes
 * -------------------------------------------------------------------------- */
esp_err_t httpsOtaUpdate(const char* url, const char* serverCertPemStart);


/* Function Definitions
 * -------------------------------------------------------------------------- */
esp_err_t httpsOtaUpdate(const char* url, const char* serverCertPemStart)
{
    ESP_LOGI(TAG, "Starting OTA update process");

    esp_err_t err = ESP_OK;

}

#endif /* __ma_jobOTA__ */
