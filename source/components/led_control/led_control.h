#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief Predefined RGB LED colors
 */
typedef enum {
    RGB_COLOR_OFF = 0,
    RGB_COLOR_RED,
    RGB_COLOR_GREEN,
    RGB_COLOR_BLUE,
    RGB_COLOR_WHITE,
    RGB_COLOR_YELLOW,
    RGB_COLOR_CYAN,
    RGB_COLOR_MAGENTA
} rgb_color_t;

/**
 * @brief Initialize RGB LED
 * 
 * @return ESP_OK on success
 */
esp_err_t led_control_init(void);

/**
 * @brief Set RGB LED color using RGB values
 * 
 * @param red Red component (0-255)
 * @param green Green component (0-255)
 * @param blue Blue component (0-255)
 * @return ESP_OK on success
 */
esp_err_t led_control_set_rgb(uint8_t red, uint8_t green, uint8_t blue);

/**
 * @brief Set RGB LED to a predefined color
 * 
 * @param color Predefined color from rgb_color_t enum
 * @return ESP_OK on success
 */
esp_err_t led_control_set_color(rgb_color_t color);


#ifdef __cplusplus
}
#endif