#include "mux_control.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "sdkconfig.h"

static const char *TAG = "mux_control";

static const gpio_num_t mux_gpios[4] = {
    CONFIG_MUX_S0_GPIO,
    CONFIG_MUX_S1_GPIO,
    CONFIG_MUX_S2_GPIO,
    CONFIG_MUX_S3_GPIO
};

static uint8_t current_channel = 0;
static bool is_enabled = false;

esp_err_t mux_control_init(void)
{
    ESP_LOGI(TAG, "Initializing multiplexer control");
    
    // Configure all mux control pins as outputs
    for (int i = 0; i < 4; i++) {
        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << mux_gpios[i]),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE
        };
        
        esp_err_t ret = gpio_config(&io_conf);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Error configuring MUX GPIO %d: %d", mux_gpios[i], ret);
            
            // Cleanup previously configured pins
            for (int j = 0; j < i; j++) {
                // Reset pins to input mode (safer state)
                gpio_config_t cleanup_conf = {
                    .pin_bit_mask = (1ULL << mux_gpios[j]),
                    .mode = GPIO_MODE_INPUT,
                    .pull_up_en = GPIO_PULLUP_DISABLE,
                    .pull_down_en = GPIO_PULLDOWN_DISABLE,
                    .intr_type = GPIO_INTR_DISABLE
                };
                gpio_config(&cleanup_conf);
            }
            return ret;
        }
        
        // Initial state: LOW
        gpio_set_level(mux_gpios[i], 0);
    }
    
    is_enabled = true;
    // Select channel 0 by default
    return mux_control_select(0);
}

esp_err_t mux_control_select(uint8_t channel)
{
    if (channel >= 16) {
        ESP_LOGE(TAG, "Invalid multiplexer channel: %d", channel);
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!is_enabled) {
        ESP_LOGW(TAG, "Multiplexer is disabled, enable it first");
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "Selecting mux channel: %d", channel);
    
    // Set the address lines according to the binary value of the channel
    for (int i = 0; i < 4; i++) {
        bool bit_value = (channel >> i) & 0x01;
        esp_err_t ret = gpio_set_level(mux_gpios[i], bit_value);
        if (ret != ESP_OK) {
            return ret;
        }
    }
    
    current_channel = channel;
    return ESP_OK;
}

uint8_t mux_control_get_channel(void)
{
    return current_channel;
}

esp_err_t mux_control_enable(void)
{
    if (is_enabled) {
        return ESP_OK;
    }
    
    is_enabled = true;
    ESP_LOGI(TAG, "Multiplexer enabled");
    
    // Restore the last channel selection
    return mux_control_select(current_channel);
}

esp_err_t mux_control_disable(void)
{
    if (!is_enabled) {
        return ESP_OK;
    }
    
    // Set all control pins to LOW
    for (int i = 0; i < 4; i++) {
        esp_err_t ret = gpio_set_level(mux_gpios[i], 0);
        if (ret != ESP_OK) {
            return ret;
        }
    }
    
    is_enabled = false;
    ESP_LOGI(TAG, "Multiplexer disabled");
    return ESP_OK;
}

bool mux_control_is_enabled(void)
{
    return is_enabled;
}
