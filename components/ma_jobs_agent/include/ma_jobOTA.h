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

/* Enums and Structs
 * -------------------------------------------------------------------------- */
typedef enum otaReturn {
    MA_OTA_ERR = -4,
    MA_OTA_DUP_FW = -3,
    MA_OTA_INV_FW = -2,
    MA_OTA_HTTP_ERROR = -1,
    MA_OTA_SUCCESS = 0
} ma_ota_err_t;


/* Function Prototypes
 * -------------------------------------------------------------------------- */
ma_ota_err_t httpsOtaUpdate(const char* url, const uint8_t* serverCertPemStart);


#endif /* __ma_jobOTA__ */
