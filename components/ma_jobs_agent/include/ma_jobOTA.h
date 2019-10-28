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

#include "esp_err.h"

/* Function Prototypes
 * -------------------------------------------------------------------------- */
esp_err_t httpsOtaUpdate(const char* url, const uint8_t* serverCertPemStart, const uint8_t* clientKeyPemStart);


#endif /* __ma_jobOTA__ */
