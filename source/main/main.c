#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include "sdkconfig.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_netif.h"              // Add this for network interface definitions
#include "esp_netif_ip_addr.h"      // Add this for IP2STR and IPSTR macros

// Project components
#include "ade7880.h"
#include "ethernet_init.h"
#include "mux_control.h"
#include "led_control.h"
#include "log_handler.h"
#include "http_service.h"
#include "network_events.h"         // Add this for NETWORK_EVENT definitions

static const char *TAG = "main";

// MUX scan configuration
#define MUX_INITIAL_SCAN_ENABLED     CONFIG_MUX_INITIAL_SCAN_ENABLED
#define MUX_INITIAL_SCAN_INTERVAL_MS CONFIG_MUX_INITIAL_SCAN_INTERVAL_MS
#define MUX_FINAL_CHANNEL            CONFIG_MUX_FINAL_CHANNEL

// Event handler for network events to start HTTP server
static void on_network_event(void* arg, esp_event_base_t event_base,
                            int32_t event_id, void* event_data)
{
    if (event_base == NETWORK_EVENT && event_id == NETWORK_EVENT_GOT_IP) {
        esp_netif_ip_info_t* ip_info = (esp_netif_ip_info_t*) event_data;
        ESP_LOGI(TAG, "Ethernet Got IP: " IPSTR, IP2STR(&ip_info->ip));
        
        // Initialize the HTTP server component directly
        ESP_LOGI(TAG, "Starting HTTP server");
        esp_err_t ret = http_server_init();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "HTTP server initialization failed: %s", esp_err_to_name(ret));
        }
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "EnergyMe - Industry");
    
    // Initialize LED control
    esp_err_t ret = led_control_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LED initialization failed");
        return;
    }
    
    // Show initializing state with green light
    led_control_set_color(RGB_COLOR_GREEN);
    
    // Initialize multiplexer control
    ret = mux_control_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Multiplexer initialization failed");
        led_control_set_color(RGB_COLOR_RED);  // Show error
        return;
    }
    
    // Initialize Ethernet - this creates the default event loop
    ret = eth_spi_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Ethernet initialization failed");
        led_control_set_color(RGB_COLOR_RED);  // Show error
        return;
    }
    
    // Register for network events - moved after Ethernet init that creates the event loop
    ESP_ERROR_CHECK(esp_event_handler_register(NETWORK_EVENT, NETWORK_EVENT_GOT_IP, &on_network_event, NULL));

    // Initialize log handler to register for network events
    // Note: It will activate logging automatically when network is connected
    ret = log_handler_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Log handler initialization failed, continuing without UDP logging");
    }
    
    // Testing some warning and error messages
    ESP_LOGW(TAG, "This is a warning message");
    ESP_LOGE(TAG, "This is an error message");

    // Initialize ADE7880 power meter
    spi_device_handle_t ade7880_spi_handle;
    ret = ade7880_init(&ade7880_spi_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ADE7880 initialization failed");
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
        led_control_set_rgb(0, 20, 0);  // Dim green

        // Delay before next reading
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}