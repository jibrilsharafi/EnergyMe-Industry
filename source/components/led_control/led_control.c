#include "led_control.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "driver/ledc.h"

static const char *TAG = "led_control";

// RGB LED GPIO pins - configured via menuconfig
#define RGB_RED_GPIO    CONFIG_RGB_LED_RED_GPIO
#define RGB_GREEN_GPIO  CONFIG_RGB_LED_GREEN_GPIO
#define RGB_BLUE_GPIO   CONFIG_RGB_LED_BLUE_GPIO

// LEDC configuration
#define LED_TIMER       CONFIG_RGB_LED_TIMER
#define LED_MODE        CONFIG_RGB_LED_MODE
#define LED_DUTY_RES    CONFIG_RGB_LED_DUTY_RES
#define LED_FREQUENCY   CONFIG_RGB_LED_FREQUENCY

#define RGB_CHANNEL_NUM 3

static const uint8_t rgb_channels[RGB_CHANNEL_NUM] = {
    LEDC_CHANNEL_0,  // Red
    LEDC_CHANNEL_1,  // Green
    LEDC_CHANNEL_2   // Blue
};

static const gpio_num_t rgb_gpios[RGB_CHANNEL_NUM] = {
    RGB_RED_GPIO,
    RGB_GREEN_GPIO,
    RGB_BLUE_GPIO
};

// Maximum duty cycle value based on resolution
static uint32_t max_duty;

esp_err_t led_control_init(void)
{
    ESP_LOGI(TAG, "Initializing RGB LED");
    
    // Calculate max duty cycle value based on resolution
    max_duty = (1ULL << LED_DUTY_RES) - 1;
    
    // Configure LEDC timer
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LED_MODE,
        .timer_num = LED_TIMER,
        .duty_resolution = LED_DUTY_RES,
        .freq_hz = LED_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK
    };
    
    esp_err_t ret = ledc_timer_config(&ledc_timer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error configuring LEDC timer: %d", ret);
        return ret;
    }
    
    // Configure LEDC channels for RGB
    for (int i = 0; i < RGB_CHANNEL_NUM; i++) {
        ledc_channel_config_t ledc_channel = {
            .speed_mode = LED_MODE,
            .channel = rgb_channels[i],
            .timer_sel = LED_TIMER,
            .intr_type = LEDC_INTR_DISABLE,
            .gpio_num = rgb_gpios[i],
            .duty = 0, // Initially off
            .hpoint = 0
        };
        
        ret = ledc_channel_config(&ledc_channel);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Error configuring LEDC channel %d: %d", i, ret);
            return ret;
        }
    }
    
    // Initialize with LED off
    ret = led_control_set_rgb(0, 0, 0);
    
    return ret;
}

esp_err_t led_control_set_rgb(uint8_t red, uint8_t green, uint8_t blue)
{
    ESP_LOGD(TAG, "Setting RGB LED to (%d, %d, %d)", red, green, blue);
    
    // Convert 8-bit color values to duty cycle values
    uint32_t red_duty = (red * max_duty) / 255;
    uint32_t green_duty = (green * max_duty) / 255;
    uint32_t blue_duty = (blue * max_duty) / 255;
    
    esp_err_t ret;
    
    ret = ledc_set_duty(LED_MODE, rgb_channels[0], red_duty);
    if (ret != ESP_OK) return ret;
    ret = ledc_update_duty(LED_MODE, rgb_channels[0]);
    if (ret != ESP_OK) return ret;
    
    ret = ledc_set_duty(LED_MODE, rgb_channels[1], green_duty);
    if (ret != ESP_OK) return ret;
    ret = ledc_update_duty(LED_MODE, rgb_channels[1]);
    if (ret != ESP_OK) return ret;
    
    ret = ledc_set_duty(LED_MODE, rgb_channels[2], blue_duty);
    if (ret != ESP_OK) return ret;
    ret = ledc_update_duty(LED_MODE, rgb_channels[2]);
    if (ret != ESP_OK) return ret;
    
    return ESP_OK;
}

esp_err_t led_control_set_color(rgb_color_t color)
{
    switch (color) {
        case RGB_COLOR_OFF:
            return led_control_set_rgb(0, 0, 0);
        case RGB_COLOR_RED:
            return led_control_set_rgb(255, 0, 0);
        case RGB_COLOR_GREEN:
            return led_control_set_rgb(0, 255, 0);
        case RGB_COLOR_BLUE:
            return led_control_set_rgb(0, 0, 255);
        case RGB_COLOR_WHITE:
            return led_control_set_rgb(255, 255, 255);
        case RGB_COLOR_YELLOW:
            return led_control_set_rgb(255, 255, 0);
        case RGB_COLOR_CYAN:
            return led_control_set_rgb(0, 255, 255);
        case RGB_COLOR_MAGENTA:
            return led_control_set_rgb(255, 0, 255);
        default:
            ESP_LOGE(TAG, "Invalid color: %d", color);
            return ESP_ERR_INVALID_ARG;
    }
}
