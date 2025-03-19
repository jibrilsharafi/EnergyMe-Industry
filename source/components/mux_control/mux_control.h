#pragma once

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize 4-to-16 multiplexer control pins
 * 
 * This function configures the GPIO pins used to control
 * a 4-to-16 multiplexer (e.g., CD74HC4067)
 * 
 * @return ESP_OK on success
 */
esp_err_t mux_control_init(void);

/**
 * @brief Select a specific multiplexer channel
 * 
 * @param channel Channel number to select (0-15)
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if channel is invalid,
 *         ESP_ERR_INVALID_STATE if multiplexer is disabled
 */
esp_err_t mux_control_select(uint8_t channel);

/**
 * @brief Get currently selected multiplexer channel
 * 
 * @return Currently selected channel
 */
uint8_t mux_control_get_channel(void);

/**
 * @brief Enable the multiplexer
 * 
 * Restores the last used channel configuration
 * 
 * @return ESP_OK on success
 */
esp_err_t mux_control_enable(void);

/**
 * @brief Disable the multiplexer
 * 
 * Sets all control pins to LOW
 * 
 * @return ESP_OK on success
 */
esp_err_t mux_control_disable(void);

/**
 * @brief Check if multiplexer is enabled
 * 
 * @return true if enabled, false if disabled
 */
bool mux_control_is_enabled(void);


#ifdef __cplusplus
}
#endif