#pragma once

#include "esp_err.h"
#include "esp_http_server.h"

/**
 * @brief Initialize the OTA service
 * 
 * Sets up the HTTP server endpoints for OTA update handling
 * 
 * @param server An existing HTTP server instance or NULL to create a new one
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t ota_service_init(httpd_handle_t server);

/**
 * @brief Start the OTA update process
 * 
 * Downloads and installs firmware from the provided URL
 * 
 * @param firmware_url URL to the firmware binary
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t ota_start_update(const char *firmware_url);