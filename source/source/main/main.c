/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

static const char *TAG = "main";

// LED configuration
#define LEDC_TIMER          LEDC_TIMER_0
#define LEDC_MODE           LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO      8 // Define the output GPIO for the LED
#define LEDC_CHANNEL        LEDC_CHANNEL_0
#define LEDC_DUTY_RES       LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_MAX_DUTY       (8191) // Maximum duty cycle (2^13 - 1)
#define LEDC_FREQUENCY      5000 // Frequency in Hz

// ADE7880 GPIO pins
#define ADE7880_RESET_PIN     GPIO_NUM_3
#define ADE7880_CS_PIN        GPIO_NUM_9
#define ADE7880_MOSI_PIN      GPIO_NUM_10
#define ADE7880_MISO_PIN      GPIO_NUM_11
#define ADE7880_SCLK_PIN      GPIO_NUM_12

// ADE7880 registers
#define ADE7880_VERSION_REG   0xE707   // Version register
#define ADE7880_AIRMS         0x43C0   // Phase A current RMS
#define ADE7880_AVRMS         0x43C1   // Phase A voltage RMS
#define ADE7880_BIRMS         0x43C2   // Phase B current RMS
#define ADE7880_BVRMS         0x43C3   // Phase B voltage RMS
#define ADE7880_CIRMS         0x43C4   // Phase C current RMS
#define ADE7880_CVRMS         0x43C5   // Phase C voltage RMS

// SPI configuration
#define SPI_HOST             SPI2_HOST
#define SPI_CLOCK_SPEED_HZ   1000000  // 1 MHz

// LED status patterns
typedef enum {
    LED_STATUS_INIT,       // Initialization - slow blink
    LED_STATUS_SUCCESS,    // Success - solid on
    LED_STATUS_ERROR,      // Error - fast blink
    LED_STATUS_WORKING     // Working - breathing pattern
} led_status_t;

static void led_init(void)
{
    // LED PWM Timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    // LED PWM Channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_IO,
        .duty           = 0, // Start with LED off
        .hpoint         = 0
    };
    ledc_channel_config(&ledc_channel);

    // Initialize fade service
    ledc_fade_func_install(0);
}

// Set the LED status pattern
static void set_led_status(led_status_t status)
{
    switch (status) {
        case LED_STATUS_INIT:
            // Slow blinking - on for 1s, off for 1s
            ESP_LOGI(TAG, "LED status: INIT (slow blinking)");
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_MAX_DUTY);
            ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0);
            ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            break;
            
        case LED_STATUS_SUCCESS:
            // Solid on
            ESP_LOGI(TAG, "LED status: SUCCESS (solid on)");
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_MAX_DUTY);
            ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
            break;
            
        case LED_STATUS_ERROR:
            // Fast blinking - on for 200ms, off for 200ms
            ESP_LOGI(TAG, "LED status: ERROR (fast blinking)");
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_MAX_DUTY);
            ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
            vTaskDelay(200 / portTICK_PERIOD_MS);
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0);
            ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
            vTaskDelay(200 / portTICK_PERIOD_MS);
            break;
            
        case LED_STATUS_WORKING:
            // Breathing effect - fade in and out
            ESP_LOGI(TAG, "LED status: WORKING (breathing)");
            ledc_set_fade_with_time(LEDC_MODE, LEDC_CHANNEL, LEDC_MAX_DUTY, 1000);
            ledc_fade_start(LEDC_MODE, LEDC_CHANNEL, LEDC_FADE_NO_WAIT);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            ledc_set_fade_with_time(LEDC_MODE, LEDC_CHANNEL, 0, 1000);
            ledc_fade_start(LEDC_MODE, LEDC_CHANNEL, LEDC_FADE_NO_WAIT);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            break;
    }
}

static void ade7880_reset(void)
{
    // Configure RESET pin as output
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << ADE7880_RESET_PIN),
        .pull_down_en = 0,
        .pull_up_en = 0
    };
    gpio_config(&io_conf);
    
    // Reset sequence
    ESP_LOGI(TAG, "Resetting ADE7880");
    gpio_set_level(ADE7880_RESET_PIN, 0);    // Assert reset (active low)
    vTaskDelay(10 / portTICK_PERIOD_MS);     // Hold for 10ms
    gpio_set_level(ADE7880_RESET_PIN, 1);    // Release reset
    vTaskDelay(100 / portTICK_PERIOD_MS);    // Wait for device to initialize
}

static void toggle_cs_pin(void)
{
    ESP_LOGI(TAG, "Toggling CS pin");
    for (int i = 0; i < 3; i++) {
        gpio_set_level(ADE7880_CS_PIN, 1);   // CS high
        vTaskDelay(10 / portTICK_PERIOD_MS);
        gpio_set_level(ADE7880_CS_PIN, 0);   // CS low
        vTaskDelay(10 / portTICK_PERIOD_MS);
        gpio_set_level(ADE7880_CS_PIN, 1);   // CS high
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

static esp_err_t spi_init(spi_device_handle_t *spi_handle)
{
    ESP_LOGI(TAG, "Initializing SPI bus");
    
    // Initialize SPI bus
    spi_bus_config_t bus_config = {
        .mosi_io_num = ADE7880_MOSI_PIN,
        .miso_io_num = ADE7880_MISO_PIN,
        .sclk_io_num = ADE7880_SCLK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32,
    };
    
    esp_err_t ret = spi_bus_initialize(SPI_HOST, &bus_config, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Configure CS pin as GPIO
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << ADE7880_CS_PIN),
        .pull_down_en = 0,
        .pull_up_en = 0
    };
    gpio_config(&io_conf);
    gpio_set_level(ADE7880_CS_PIN, 1);  // Start with CS high
    
    // Configure SPI device
    spi_device_interface_config_t dev_config = {
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .mode = 0,                   // SPI mode 0
        .clock_speed_hz = SPI_CLOCK_SPEED_HZ,
        .spics_io_num = -1,          // We'll manage CS pin manually
        .queue_size = 1,
    };
    
    ret = spi_bus_add_device(SPI_HOST, &dev_config, spi_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add SPI device: %s", esp_err_to_name(ret));
        return ret;
    }
    
    return ESP_OK;
}

// General function to read a register from ADE7880
static esp_err_t ade7880_read_register(spi_device_handle_t spi_handle, uint16_t reg_addr, uint32_t *value)
{
    // For reading 24/32-bit registers, we need 4 bytes for command and 4 bytes for data
    uint8_t tx_data[8] = {0};
    uint8_t rx_data[8] = {0};
    
    // Prepare command to read register
    tx_data[0] = 0x01;                      // Read operation
    tx_data[1] = (reg_addr >> 8) & 0xFF;    // High byte of address
    tx_data[2] = reg_addr & 0xFF;           // Low byte of address
    
    // Configure SPI transaction
    spi_transaction_t t = {
        .length = 8 * 8,             // 8 bytes (64 bits)
        .tx_buffer = tx_data,
        .rx_buffer = rx_data,
    };
    
    // Execute transaction
    ESP_LOGD(TAG, "Reading register 0x%04X", reg_addr);
    gpio_set_level(ADE7880_CS_PIN, 0);  // Assert CS
    esp_err_t ret = spi_device_polling_transmit(spi_handle, &t);
    gpio_set_level(ADE7880_CS_PIN, 1);  // Deassert CS
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI transaction failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // For 24-bit registers stored in 32 bits:
    // The data is returned in rx_data[4:7]
    *value = ((uint32_t)rx_data[4] << 24) |
             ((uint32_t)rx_data[5] << 16) |
             ((uint32_t)rx_data[6] << 8)  |
             ((uint32_t)rx_data[7]);
    
    return ESP_OK;
}

// Function to read version register (8-bit)
static esp_err_t read_version_register(spi_device_handle_t spi_handle, uint8_t *version)
{
    uint32_t value = 0;
    esp_err_t ret = ade7880_read_register(spi_handle, ADE7880_VERSION_REG, &value);
    if (ret == ESP_OK) {
        *version = (uint8_t)(value & 0xFF);
    }
    return ret;
}

// Function to read and print all RMS values
static void read_rms_values(spi_device_handle_t spi_handle)
{
    uint32_t value;
    int32_t signed_value;
    
    // Phase A current RMS
    if (ade7880_read_register(spi_handle, ADE7880_AIRMS, &value) == ESP_OK) {
        signed_value = (int32_t)value;  // These are signed values
        ESP_LOGI(TAG, "Phase A Current RMS: %"PRId32" (0x%08"PRIX32")", signed_value, value);
    }
    
    // Phase A voltage RMS
    if (ade7880_read_register(spi_handle, ADE7880_AVRMS, &value) == ESP_OK) {
        signed_value = (int32_t)value;
        ESP_LOGI(TAG, "Phase A Voltage RMS: %"PRId32" (0x%08"PRIX32")", signed_value, value);
    }
    
    // Phase B current RMS
    if (ade7880_read_register(spi_handle, ADE7880_BIRMS, &value) == ESP_OK) {
        signed_value = (int32_t)value;
        ESP_LOGI(TAG, "Phase B Current RMS: %"PRId32" (0x%08"PRIX32")", signed_value, value);
    }
    
    // Phase B voltage RMS
    if (ade7880_read_register(spi_handle, ADE7880_BVRMS, &value) == ESP_OK) {
        signed_value = (int32_t)value;
        ESP_LOGI(TAG, "Phase B Voltage RMS: %"PRId32" (0x%08"PRIX32")", signed_value, value);
    }
    
    // Phase C current RMS
    if (ade7880_read_register(spi_handle, ADE7880_CIRMS, &value) == ESP_OK) {
        signed_value = (int32_t)value;
        ESP_LOGI(TAG, "Phase C Current RMS: %"PRId32" (0x%08"PRIX32")", signed_value, value);
    }
    
    // Phase C voltage RMS
    if (ade7880_read_register(spi_handle, ADE7880_CVRMS, &value) == ESP_OK) {
        signed_value = (int32_t)value;
        ESP_LOGI(TAG, "Phase C Voltage RMS: %"PRId32" (0x%08"PRIX32")", signed_value, value);
    }
    
    // Print separator for readability
    ESP_LOGI(TAG, "--------------------------------");
}

void app_main(void)
{
    // Set global logging level to debug
    esp_log_level_set("*", ESP_LOG_DEBUG);
    
    ESP_LOGI(TAG, "ADE7880 SPI Connection Test");
    
    // Initialize LED PWM controller
    led_init();
    
    // Show initialization status
    set_led_status(LED_STATUS_INIT);

    // Initialize SPI for ADE7880
    spi_device_handle_t spi_handle;
    esp_err_t ret = spi_init(&spi_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI initialization failed");
        while (1) {
            set_led_status(LED_STATUS_ERROR);
        }
    }
    
    // Reset ADE7880
    ade7880_reset();
    
    // Toggle CS pin three times
    toggle_cs_pin();
    
    // Show working status
    set_led_status(LED_STATUS_WORKING);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    
    // Read version register
    uint8_t version = 0;
    ret = read_version_register(spi_handle, &version);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read version register");
        while (1) {
            set_led_status(LED_STATUS_ERROR);
        }
    }
    
    // Check if we got a valid version
    ESP_LOGI(TAG, "ADE7880 Version: 0x%02X", version);
    
    // Expected version is typically 0x02 for ADE7880, but check your device documentation
    if (version != 0) {
        ESP_LOGI(TAG, "Successfully communicated with ADE7880");
        
        // Main loop - continuously read RMS values
        while (1) {
            set_led_status(LED_STATUS_SUCCESS);
            
            // Read all RMS values
            read_rms_values(spi_handle);
            
            // Delay before next reading
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
    } else {
        ESP_LOGE(TAG, "Invalid version read. SPI communication failed");
        while (1) {
            set_led_status(LED_STATUS_ERROR);
        }
    }
}