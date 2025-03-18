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

#include "ade7880_registers.h"

static const char *TAG = "main";

// Define W5500 SPI pins
#define W5500_SPI_HOST     SPI3_HOST
#define W5500_SPI_MISO     19
#define W5500_SPI_MOSI     23
#define W5500_SPI_SCLK     18
#define W5500_SPI_CS       5
#define W5500_INT_PIN      4
#define W5500_RST_PIN      15
#define W5500_SPI_CLOCK_SPEED_HZ 40000000 // 40 MHz

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

// Multiplexer select pins
#define MUX_S0_PIN     GPIO_NUM_35  // Physical pin 38
#define MUX_S1_PIN     GPIO_NUM_36  // Physical pin 37
#define MUX_S2_PIN     GPIO_NUM_37  // Physical pin 36
#define MUX_S3_PIN     GPIO_NUM_38  // Physical pin 35

// Add these configuration defines at the top of the file, under the existing defines
#define MUX_INITIAL_SCAN_ENABLED     1       // 1 to enable initial scan, 0 to disable
#define MUX_INITIAL_SCAN_INTERVAL_MS 1000    // Time between channels during scan (ms)
#define MUX_FINAL_CHANNEL            0       // Channel to stay on after scan completes

// SPI configuration
#define SPI_HOST             SPI2_HOST
#define SPI_CLOCK_SPEED_HZ   2500000  // 2.5 MHz

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

// Initialize the multiplexer pins
static void mux_init(void)
{
    // Configure multiplexer select pins as outputs
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << MUX_S0_PIN) | 
                         (1ULL << MUX_S1_PIN) | 
                         (1ULL << MUX_S2_PIN) | 
                         (1ULL << MUX_S3_PIN),
        .pull_down_en = 0,
        .pull_up_en = 0
    };
    gpio_config(&io_conf);
    
    // Initialize all select pins to 0 (channel 0)
    gpio_set_level(MUX_S0_PIN, 0);
    gpio_set_level(MUX_S1_PIN, 0);
    gpio_set_level(MUX_S2_PIN, 0);
    gpio_set_level(MUX_S3_PIN, 0);
    
    ESP_LOGI(TAG, "Multiplexers initialized to channel 0");
}


// Set the multiplexer channel (0-15)
static void set_mux_channel(uint8_t channel)
{
    // Ensure channel is between 0-15
    channel &= 0x0F;
    
    // Set the select pins according to the channel value
    gpio_set_level(MUX_S0_PIN, channel & 0x01);
    ESP_LOGD(TAG, "MUX_S0_PIN: %d", channel & 0x01);
    gpio_set_level(MUX_S1_PIN, (channel >> 1) & 0x01);
    ESP_LOGD(TAG, "MUX_S1_PIN: %d", (channel >> 1) & 0x01);
    gpio_set_level(MUX_S2_PIN, (channel >> 2) & 0x01);
    ESP_LOGD(TAG, "MUX_S2_PIN: %d", (channel >> 2) & 0x01);
    gpio_set_level(MUX_S3_PIN, (channel >> 3) & 0x01);
    ESP_LOGD(TAG, "MUX_S3_PIN: %d", (channel >> 3) & 0x01);
    
    ESP_LOGI(TAG, "Switched multiplexers to channel %d", channel);
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

// Updated function to write to a register using the register information struct
static esp_err_t ade7880_write_register(spi_device_handle_t spi_handle, 
                                       const ade7880_reg_t *reg, 
                                       uint32_t value)
{
    // Check if register is writable
    if (!reg->writable) {
        ESP_LOGE(TAG, "Register %s (0x%04X) is not writable", reg->name, reg->address);
        return ESP_ERR_INVALID_ARG;
    }
    
    uint8_t tx_data[7] = {0};
    
    // Prepare command
    tx_data[0] = 0x00;                   // Write operation
    tx_data[1] = (reg->address >> 8) & 0xFF; // High byte of address
    tx_data[2] = reg->address & 0xFF;        // Low byte of address
    
    // Set up data portion based on communication length
    if (reg->comm_length == 32) {
        tx_data[3] = (value >> 24) & 0xFF;
        tx_data[4] = (value >> 16) & 0xFF;
        tx_data[5] = (value >> 8) & 0xFF;
        tx_data[6] = value & 0xFF;
    } else if (reg->comm_length == 16) {
        tx_data[3] = (value >> 8) & 0xFF;
        tx_data[4] = value & 0xFF;
    } else { // 8-bit
        tx_data[3] = value & 0xFF;
    }
    
    // Calculate transaction length in bytes (command byte + address bytes + data bytes)
    int tx_bytes = 3 + (reg->comm_length / 8);
    
    // Configure SPI transaction
    spi_transaction_t t = {
        .length = tx_bytes * 8,
        .tx_buffer = tx_data,
        .rx_buffer = NULL,
    };
    
    // Execute transaction
    ESP_LOGD(TAG, "Writing register %s (0x%04X) with value 0x%08" PRIx32, 
             reg->name, reg->address, value);
    
    gpio_set_level(ADE7880_CS_PIN, 0);
    esp_err_t ret = spi_device_polling_transmit(spi_handle, &t);
    gpio_set_level(ADE7880_CS_PIN, 1);
    
    return ret;
}

// Updated function to read from a register using the register information struct
static esp_err_t ade7880_read_register(spi_device_handle_t spi_handle, 
                                      const ade7880_reg_t *reg, 
                                      uint32_t *value)
{
    uint8_t tx_data[7] = {0};
    uint8_t rx_data[7] = {0};
    
    // Prepare command
    tx_data[0] = 0x01;                        // Read operation
    tx_data[1] = (reg->address >> 8) & 0xFF;  // High byte of address
    tx_data[2] = reg->address & 0xFF;         // Low byte of address
    
    // Calculate total bytes to transact (command + address + data)
    int total_bytes = 3 + (reg->comm_length / 8);
    
    // Configure SPI transaction
    spi_transaction_t t = {
        .length = total_bytes * 8,
        .tx_buffer = tx_data,
        .rx_buffer = rx_data,
    };
    
    // Execute transaction
    ESP_LOGD(TAG, "Reading register %s (0x%04X)", reg->name, reg->address);
    
    gpio_set_level(ADE7880_CS_PIN, 0);
    esp_err_t ret = spi_device_polling_transmit(spi_handle, &t);
    gpio_set_level(ADE7880_CS_PIN, 1);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI transaction failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Extract the value based on communication length
    if (reg->comm_length == 32) {
        // This creates the correct byte ordering for 32-bit reads
        *value = ((uint32_t)rx_data[3] << 24) |
                 ((uint32_t)rx_data[4] << 16) |
                 ((uint32_t)rx_data[5] << 8)  |
                 ((uint32_t)rx_data[6]);
                 
        ESP_LOGD(TAG, "Raw read value: 0x%08" PRIx32, *value);
    } else if (reg->comm_length == 16) {
        *value = ((uint32_t)rx_data[3] << 8) | (uint32_t)rx_data[4];
    } else { // 8-bit
        *value = (uint32_t)rx_data[3];
    }
    
    return ESP_OK;
}

// Function to read version register with updated API
static esp_err_t read_version_register(spi_device_handle_t spi_handle, uint8_t *version)
{
    uint32_t value = 0;
    esp_err_t ret = ade7880_read_register(spi_handle, &ADE7880_VERSION_REG, &value);
    if (ret == ESP_OK) {
        *version = (uint8_t)(value & 0xFF);
    }
    return ret;
}

// Updated read_rms_values function using the new structs
static void read_rms_values(spi_device_handle_t spi_handle)
{
    uint32_t value;
    int32_t signed_value;
    char phase_currents[128] = {0};
    char phase_voltages[128] = {0};
    
    int pos_current = sprintf(phase_currents, "Currents: ");
    int pos_voltage = sprintf(phase_voltages, "Voltages: ");
    
    // Phase A current RMS
    if (ade7880_read_register(spi_handle, &ADE7880_AIRMS, &value) == ESP_OK) {
        signed_value = (int32_t)value;
        pos_current += sprintf(phase_currents + pos_current, "A=%ld, ", signed_value);
    }
    
    // Phase A voltage RMS
    if (ade7880_read_register(spi_handle, &ADE7880_AVRMS, &value) == ESP_OK) {
        signed_value = (int32_t)value;
        pos_voltage += sprintf(phase_voltages + pos_voltage, "A=%ld, ", signed_value);
    }
    
    // Phase B current RMS
    if (ade7880_read_register(spi_handle, &ADE7880_BIRMS, &value) == ESP_OK) {
        signed_value = (int32_t)value;
        pos_current += sprintf(phase_currents + pos_current, "B=%ld, ", signed_value);
    }
    
    // Phase B voltage RMS
    if (ade7880_read_register(spi_handle, &ADE7880_BVRMS, &value) == ESP_OK) {
        signed_value = (int32_t)value;
        pos_voltage += sprintf(phase_voltages + pos_voltage, "B=%ld, ", signed_value);
    }
    
    // Phase C current RMS
    if (ade7880_read_register(spi_handle, &ADE7880_CIRMS, &value) == ESP_OK) {
        signed_value = (int32_t)value;
        pos_current += sprintf(phase_currents + pos_current, "C=%ld", signed_value);
    }
    
    // Phase C voltage RMS
    if (ade7880_read_register(spi_handle, &ADE7880_CVRMS, &value) == ESP_OK) {
        signed_value = (int32_t)value;
        pos_voltage += sprintf(phase_voltages + pos_voltage, "C=%ld", signed_value);
    }
    
    // Print the concise lines
    ESP_LOGI(TAG, "%s", phase_currents);
    ESP_LOGI(TAG, "%s", phase_voltages);
    ESP_LOGI(TAG, "--------------------------------");
}

// Updated test_register_write_read function using the new structs
static esp_err_t test_register_write_read(spi_device_handle_t spi_handle)
{
    esp_err_t ret;
    uint32_t test_value = 0x123456;  // Test value to write (24-bit)
    uint32_t read_value;
    
    // Test registers to write and read
    const ade7880_reg_t* test_registers[] = {
        &ADE7880_AIGAIN,
        &ADE7880_AVGAIN,
        &ADE7880_BIGAIN,
        &ADE7880_BVGAIN,
        &ADE7880_CIGAIN,
        &ADE7880_CVGAIN
    };
    
    // Test each register
    for (int i = 0; i < sizeof(test_registers) / sizeof(test_registers[0]); i++) {
        const ade7880_reg_t* reg = test_registers[i];
        
        // Write test value to register
        ESP_LOGI(TAG, "Writing 0x%06" PRIx32 " to %s (0x%04X)", test_value, reg->name, reg->address);
        ret = ade7880_write_register(spi_handle, reg, test_value);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to write to %s register", reg->name);
            return ret;
        }
        
        // Read back the value
        ret = ade7880_read_register(spi_handle, reg, &read_value);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read from %s register", reg->name);
            return ret;
        }
        
        // Verify the value (note: only the lower 24 bits are valid)
        ESP_LOGI(TAG, "Read 0x%06" PRIx32 " from %s (0x%04X)", read_value, reg->name, reg->address);
        if ((read_value) != test_value) {
            ESP_LOGW(TAG, "Value mismatch for %s: wrote 0x%06" PRIx32 ", read 0x%06" PRIx32 "", 
                    reg->name, test_value, read_value);
        }

        // Reset to zero for this register
        ret = ade7880_write_register(spi_handle, reg, 0x000000);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to reset %s register", reg->name);
            return ret;
        }
        
        // Use different test value for next register
        test_value = (test_value + 0x111111);  // Increment but keep to 24 bits
    }
    
    return ESP_OK;
}

static esp_err_t test_default_registers(spi_device_handle_t spi_handle)
{
    ESP_LOGI(TAG, "Testing default register values");
    
    // Array of registers to test
    const ade7880_reg_t* test_registers[] = {
        &ADE7880_LINECYC,
        &ADE7880_ZXTOUT,
        &ADE7880_COMPMODE,
        &ADE7880_GAIN,
        &ADE7880_CFMODE,
        &ADE7880_CF1DEN,
        &ADE7880_CF2DEN,
        &ADE7880_CF3DEN,
        &ADE7880_APHCAL,
        &ADE7880_BPHCAL,
        &ADE7880_CPHCAL,
        &ADE7880_PHSIGN,
        &ADE7880_CONFIG,
        &ADE7880_MMODE,
        &ADE7880_ACCMODE,
        &ADE7880_LCYCMODE,
        &ADE7880_PEAKCYC,
        &ADE7880_SAGCYC,
        &ADE7880_CFCYC,
        &ADE7880_HSDC_CFG
    };
    
    uint32_t value;
    esp_err_t ret;
    int match_count = 0;
    int total_count = sizeof(test_registers) / sizeof(test_registers[0]);
    
    // Test each register
    for (int i = 0; i < total_count; i++) {
        const ade7880_reg_t* reg = test_registers[i];
        
        // Read the register value
        ret = ade7880_read_register(spi_handle, reg, &value);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read %s register: %s", reg->name, esp_err_to_name(ret));
            continue;
        }
        
        // For 16-bit registers, mask to ignore upper bits
        if (reg->comm_length == 16) {
            value &= 0xFFFF;
        } else if (reg->comm_length == 8) {
            value &= 0xFF;
        }
        
        // Check if the value matches the default
        if (value == reg->default_value) {
            ESP_LOGI(TAG, "%s: Read 0x%04" PRIx32 " - matches expected default", 
                     reg->name, value);
            match_count++;
        } else {
            ESP_LOGW(TAG, "%s: Read 0x%04" PRIx32 " - DOES NOT match expected default 0x%04" PRIx32 "", 
                     reg->name, value, reg->default_value);
        }
    }
    
    ESP_LOGI(TAG, "Default value test complete: %d/%d registers matched defaults",
             match_count, total_count);
    
    return ESP_OK;
}

// Updated start DSP function
static esp_err_t ade7880_start_dsp(spi_device_handle_t spi_handle)
{
    ESP_LOGI(TAG, "Starting ADE7880 DSP");
    // Write 1 to RUN register to start DSP
    return ade7880_write_register(spi_handle, &ADE7880_RUN_REG, 1);
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

    mux_init();  // Initialize multiplexers

    // Current multiplexer channel
    uint8_t current_channel = 0;
    TickType_t last_channel_change = xTaskGetTickCount();

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

        ESP_LOGI(TAG, "Checking default register values...");
        ret = test_default_registers(spi_handle);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Default register test failed");
            set_led_status(LED_STATUS_ERROR);
        }
        
        // Test register write and read operations
        ret = test_register_write_read(spi_handle);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Register write/read test failed");
            set_led_status(LED_STATUS_ERROR);
        }

        // Start DSP
        ret = ade7880_start_dsp(spi_handle);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to start DSP");
            set_led_status(LED_STATUS_ERROR);
        }

        // Main loop - read RMS values and control multiplexer channels
        bool initial_scan_completed = false;

        while (1) {
            set_led_status(LED_STATUS_SUCCESS);
            
            // Read all RMS values
            read_rms_values(spi_handle);
            
            // Handle multiplexer channel cycling if initial scan is enabled
            if (MUX_INITIAL_SCAN_ENABLED && !initial_scan_completed) {
                // Get current time
                TickType_t current_time = xTaskGetTickCount();
                
                // Check if it's time to change the channel
                if (current_time - last_channel_change >= (MUX_INITIAL_SCAN_INTERVAL_MS / portTICK_PERIOD_MS)) {
                    if (current_channel < 15) {
                        // Continue scanning through channels
                        current_channel++;
                        set_mux_channel(current_channel);
                        last_channel_change = current_time;
                        ESP_LOGI(TAG, "Initial scan: now at channel %d", current_channel);
                    } else {
                        // Initial scan completed, switch to final channel
                        current_channel = MUX_FINAL_CHANNEL;
                        set_mux_channel(current_channel);
                        initial_scan_completed = true;
                        ESP_LOGI(TAG, "Initial scan completed, staying on channel %d", current_channel);
                    }
                }
            } else if (!initial_scan_completed) {
                // If initial scan is disabled, just go straight to the final channel
                current_channel = MUX_FINAL_CHANNEL;
                set_mux_channel(current_channel);
                initial_scan_completed = true;
                ESP_LOGI(TAG, "Initial scan disabled, going directly to channel %d", current_channel);
            }
            
            // Delay before next reading
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
    } else {
        ESP_LOGE(TAG, "Failed to communicate with ADE7880");
        set_led_status(LED_STATUS_ERROR);
    }
    
    // Clean up SPI bus
    spi_bus_remove_device(spi_handle);
    spi_bus_free(SPI_HOST);
    
    // Set LED to success state
    set_led_status(LED_STATUS_SUCCESS);
    
    // Delay before restart
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    // Restart the application
    esp_restart();
}