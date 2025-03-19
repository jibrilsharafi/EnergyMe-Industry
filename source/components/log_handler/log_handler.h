#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the log handler to redirect logs to UDP
 * 
 * @return ESP_OK if successful, otherwise an error code
 */
esp_err_t log_handler_init(void);

/**
 * @brief Deinitialize the log handler
 * 
 * @return ESP_OK if successful, otherwise an error code
 */
esp_err_t log_handler_deinit(void);

#ifdef __cplusplus
}
#endif
