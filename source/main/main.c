#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include "sdkconfig.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_event.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"

// Project components
#include "ade7880.h"
#include "ethernet_init.h"
#include "mux_control.h"
#include "led_control.h"

static const char *TAG = "main";

// MUX scan configuration
#define MUX_INITIAL_SCAN_ENABLED     1       // 1 to enable initial scan, 0 to disable
#define MUX_INITIAL_SCAN_INTERVAL_MS 1000    // Time between channels during scan (ms)
#define MUX_FINAL_CHANNEL            0       // Channel to stay on after scan completes

void app_main(void)
{
    ESP_LOGI(TAG, "EnergyMe Industry Power Monitor");
    
    // Initialize LED control
    esp_err_t ret = led_control_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LED initialization failed");
        return;
    }
    
    // Show initializing state with blue light
    led_control_set_color(RGB_COLOR_BLUE);
    
    // Initialize multiplexer control
    ret = mux_control_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Multiplexer initialization failed");
        led_control_set_color(RGB_COLOR_RED);  // Show error
        return;
    }
    
    // Initialize ADE7880 power meter
    spi_device_handle_t ade7880_spi_handle;
    ret = ade7880_init(&ade7880_spi_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ADE7880 initialization failed");
        led_control_set_color(RGB_COLOR_RED);  // Show error
        return;
    }
    
    // Initialize Ethernet
    ret = eth_spi_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Ethernet initialization failed");
        led_control_set_color(RGB_COLOR_RED);  // Show error
        return;
    }
    
    // Current multiplexer channel
    uint8_t current_channel = 0;
    TickType_t last_channel_change = xTaskGetTickCount();
    
    // Main loop - read RMS values and control multiplexer channels
    bool initial_scan_completed = false;
    
    // All systems initialized - show green
    led_control_set_color(RGB_COLOR_GREEN);

    while (1) {
        // Read all RMS values from ADE7880
        ade7880_read_rms_values(ade7880_spi_handle);
        
        // Handle multiplexer channel cycling if initial scan is enabled
        if (MUX_INITIAL_SCAN_ENABLED && !initial_scan_completed) {
            TickType_t current_time = xTaskGetTickCount();
            
            if (current_time - last_channel_change >= (MUX_INITIAL_SCAN_INTERVAL_MS / portTICK_PERIOD_MS)) {
                if (current_channel < 15) {
                    // Continue scanning through channels
                    current_channel++;
                    mux_control_select(current_channel);
                    last_channel_change = current_time;
                    ESP_LOGI(TAG, "Initial scan: now at channel %d", current_channel);
                } else {
                    // Initial scan completed, switch to final channel
                    current_channel = MUX_FINAL_CHANNEL;
                    mux_control_select(current_channel);
                    initial_scan_completed = true;
                    ESP_LOGI(TAG, "Initial scan completed, staying on channel %d", current_channel);
                }
            }
        } else if (!initial_scan_completed) {
            // If initial scan is disabled, just go straight to the final channel
            current_channel = MUX_FINAL_CHANNEL;
            mux_control_select(current_channel);
            initial_scan_completed = true;
            ESP_LOGI(TAG, "Initial scan disabled, going directly to channel %d", current_channel);
        }
        
        // Visual indication that system is working - blink green LED
        led_control_set_rgb(0, 255, 0);  // Full green
        vTaskDelay(100 / portTICK_PERIOD_MS);
        led_control_set_rgb(0, 100, 0);  // Dim green
        
        // Delay before next reading
        vTaskDelay(900 / portTICK_PERIOD_MS);
    }
}