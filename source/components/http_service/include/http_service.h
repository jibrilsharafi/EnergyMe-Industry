#pragma once

#include "esp_err.h"
#include "esp_http_server.h"
#include "cJSON.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Callback function type for component data
 * 
 * Components can provide this callback to add their own data
 * to the system data JSON response
 * 
 * @param json JSON object to add data to
 */
typedef void (*http_server_data_callback_t)(cJSON *json);

/**
 * @brief Initialize the HTTP server
 * 
 * @return ESP_OK on success, otherwise an error code
 */
esp_err_t http_server_init(void);

/**
 * @brief Register a component with the HTTP server
 * 
 * @param name Component name (used in JSON responses)
 * @param data_callback Callback function to provide data for the system data API
 * @param endpoints Array of endpoint definitions
 * @param endpoint_count Number of endpoints in the array
 * @return ESP_OK on success, otherwise an error code
 */
esp_err_t http_server_register_component(const char *name, 
                                         http_server_data_callback_t data_callback,
                                         const httpd_uri_t *endpoints, 
                                         size_t endpoint_count);

/**
 * @brief Start an OTA update with the given firmware URL
 * 
 * @param firmware_url URL to download the firmware from
 * @return ESP_OK on success, otherwise an error code
 */
esp_err_t ota_start_update(const char *firmware_url);

#ifdef __cplusplus
}
#endif
