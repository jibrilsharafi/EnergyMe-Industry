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

// ADE7880 gain registers
#define ADE7880_AIGAIN       0x4380   // Phase A current gain adjust
#define ADE7880_AVGAIN       0x4381   // Phase A voltage gain adjust
#define ADE7880_BIGAIN       0x4382   // Phase B current gain adjust
#define ADE7880_BVGAIN       0x4383   // Phase B voltage gain adjust
#define ADE7880_CIGAIN       0x4384   // Phase C current gain adjust
#define ADE7880_CVGAIN       0x4385   // Phase C voltage gain adjust

// ADE7880 DSP control registers
#define ADE7880_RUN_REG     0xE228   // DSP run register

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

// General function to write a register to ADE7880
static esp_err_t ade7880_write_register(spi_device_handle_t spi_handle, uint16_t reg_addr, uint32_t value)
{
    // For writing 24/32-bit registers, we need 7 bytes (1 for write command, 2 for address, 4 for data)
    uint8_t tx_data[7] = {0};
    
    // Prepare command to write register
    tx_data[0] = 0x00;                      // Write operation
    tx_data[1] = (reg_addr >> 8) & 0xFF;    // High byte of address
    tx_data[2] = reg_addr & 0xFF;           // Low byte of address
    
    // 24-bit value in 32-bit field (MSB first)
    tx_data[3] = (value >> 24) & 0xFF;      // MSB
    tx_data[4] = (value >> 16) & 0xFF;
    tx_data[5] = (value >> 8) & 0xFF;
    tx_data[6] = value & 0xFF;              // LSB
    
    // Configure SPI transaction
    spi_transaction_t t = {
        .length = 7 * 8,                    // 7 bytes (56 bits)
        .tx_buffer = tx_data,
        .rx_buffer = NULL,                  // No need to receive for write operation
    };
    
    // Execute transaction
    ESP_LOGD(TAG, "Writing register 0x%04X with value 0x%08"PRIX32, reg_addr, value);
    gpio_set_level(ADE7880_CS_PIN, 0);      // Assert CS
    esp_err_t ret = spi_device_polling_transmit(spi_handle, &t);
    gpio_set_level(ADE7880_CS_PIN, 1);      // Deassert CS
    
    return ret;
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

// Function to test register write and read operations
static esp_err_t test_register_write_read(spi_device_handle_t spi_handle)
{
    esp_err_t ret;
    uint32_t test_value = 0x123456;  // Test value to write (24-bit)
    uint32_t read_value;
    
    // Test registers to write and read
    uint16_t test_registers[] = {
        ADE7880_AIGAIN,
        ADE7880_AVGAIN,
        ADE7880_BIGAIN,
        ADE7880_BVGAIN,
        ADE7880_CIGAIN,
        ADE7880_CVGAIN
    };
    
    const char* reg_names[] = {
        "AIGAIN",
        "AVGAIN",
        "BIGAIN",
        "BVGAIN",
        "CIGAIN",
        "CVGAIN"
    };
    
    // Test each register
    for (int i = 0; i < sizeof(test_registers) / sizeof(test_registers[0]); i++) {
        // Write test value to register
        ESP_LOGI(TAG, "Writing 0x%06" PRIx32 " to %s (0x%04X)", test_value, reg_names[i], test_registers[i]);
        ret = ade7880_write_register(spi_handle, test_registers[i], test_value);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to write to %s register", reg_names[i]);
            return ret;
        }
        
        // Read back the value
        ret = ade7880_read_register(spi_handle, test_registers[i], &read_value);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read from %s register", reg_names[i]);
            return ret;
        }
        
        // Verify the value (note: only the lower 24 bits are valid)
        ESP_LOGI(TAG, "Read 0x%06" PRIx32 " from %s (0x%04X)", read_value & 0xFFFFFF, reg_names[i], test_registers[i]);
        if ((read_value & 0xFFFFFF) != test_value) {
            ESP_LOGW(TAG, "Value mismatch for %s: wrote 0x%06" PRIx32 ", read 0x%06" PRIx32 "", 
                    reg_names[i], test_value, read_value & 0xFFFFFF);
        }
        
        // Use different test value for next register
        test_value = (test_value + 0x111111) & 0xFFFFFF;  // Increment but keep to 24 bits
    }
    
    return ESP_OK;
}

static esp_err_t ade7880_start_dsp(spi_device_handle_t spi_handle)
{
    ESP_LOGI(TAG, "Starting ADE7880 DSP");
    // Write 1 to RUN register to start DSP
    return ade7880_write_register(spi_handle, ADE7880_RUN_REG, 0x0001);
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

// DIGITAL SIGNAL PROCESSOR 
// The ADE7880 contains a fixed function digital signal processor 
// (DSP) that computes all powers and rms values. It contains 
// program memory ROM and data memory RAM.  
// The program used for the power and rms computations is stored 
// in the program memory ROM and the processor executes it every 
// 8 kHz. The end of the computations is signaled by setting Bit 17 
// (DREADY) to 1 in the STATUS0 register. An interrupt attached 
// to this flag can be enabled by setting Bit 17 (DREADY) in the 
// MASK0 register. If enabled, the IRQ0 pin is set low and Status 
// Bit DREADY is set to 1 at the end of the computations. The status 
// bit is cleared and the IRQ0 pin is set to high by writing to the 
// STATUS0 register with Bit 17 (DREADY) set to 1. 
// The registers used by the DSP are located in the data memory 
// RAM, at addresses between 0x4380 and 0x43BE. The width of 
// this memory is 28 bits. A two-stage pipeline is used when write 
// operations to the data memory RAM are executed. This means 
// two things: when only one register needs to be initialized, write 
// it two more times to ensure the value has been written into RAM. 
// When two or more registers need to be initialized, write the last 
// register in the queue two more times to ensure the value is 
// written into RAM. 
// As explained in the Power-Up Procedure section, at power-up 
// or after a hardware or software reset, the DSP is in idle mode. 
// No instruction is executed. All the registers located in the data 
// memory RAM are initialized at 0, their default values and they 
// can be read/written without any restriction. The Run register, 
// used to start and stop the DSP, is cleared to 0x0000. The Run 
// register needs to be written with 0x0001 for the DSP to start 
// code execution. It is recommended to first initialize all ADE7880 
// registers located in the data memory RAM with their desired 
// values. Next, write the last register in the queue two additional 
// times to flush the pipeline and then write the Run register with 
// ADE7880 
// 0x0001. In this way, the DSP starts the computations from a 
// desired configuration.  
// To protect the integrity of the data stored in the data memory RAM 
// of the DSP (between Address 0x4380 and Address 0x43BE), a write 
// protection mechanism is available. By default, the protection is 
// disabled and registers placed between 0x4380 and 0x43BE can 
// be written without restriction. When the protection is enabled, 
// no writes to these registers are allowed. Registers can be always 
// read, without restriction, independent of the write protection 
// state. To enable the protection, write 0xAD to an internal 8-bit 
// register located at Address 0xE7FE, followed by a write of 0x80 
// to an internal 8-bit register located at Address 0xE7E3. To disable 
// the protection, write 0xAD to an internal 8-bit register located 
// at Address 0xE7FE, followed by a write of 0x00 to an internal 8-bit 
// register located at Address 0xE7E3. It is recommended to enable 
// the write protection before starting the DSP. If any data memory 
// RAM based register needs to be changed, simply disable the 
// protection, change the value and then enable back the protection. 
// There is no need to stop the DSP in order to change these registers. 
// To disable the protection, write 0xAD to an internal 8-bit register 
// located at Address 0xE7FE, followed by a write of 0x00 to an 
// internal 8-bit register located at Address 0xE7E3.  
// Use the following procedure to initialize the ADE7880 registers 
// at power-up: 
// 1. 
// 2. 
// 3. 
// 4. 
// 5. 
// 6. 
// Select the PGA gains in the phase currents, voltages, and 
// neutral current channels: Bits [2:0] (PGA1), Bits [5:3] 
// (PGA2) and Bits [8:6] (PGA3) in the Gain register. 
// If Rogowski coils are used, enable the digital integrators in 
// the phase and neutral currents: Bit 0 (INTEN) set to 1 in 
// CONFIG register. Initialize DICOEFF register to 0xFF8000 
// before setting the INTEN bit in the CONFIG register. 
// If fn is between 55 Hz and 66 Hz, set Bit 14 (SELFREQ) in 
// COMPMODE register. 
// Initialize all the other data memory RAM registers. Write 
// the last register in the queue three times to ensure that its 
// value is written into the RAM. 
// Initialize WTHR, VARTHR, VATHR, VLEVEL and 
// VNOM registers based on Equation 26, Equation 37, 
// Equation 44, Equation 22, and Equation 42, respectively. 
// Initialize CF1DEN, CF2DEN, and CF3DEN based on 
// Equation 49.  
// Data Sheet 
// 7. 
// 8. 
// 9. 
// Enable the data memory RAM protection by writing 0xAD 
// to an internal 8-bit register located at Address 0xE7FE 
// followed by a write of 0x80 to an internal 8-bit register 
// located at Address 0xE7E3. 
// Read back all data memory RAM registers to ensure that 
// they initialized with the desired values. In the unlikely case 
// that one or more registers does not initialize correctly, disable 
// the protection by writing 0xAD to an internal 8-bit register 
// located at Address 0xE7FE, followed by a write of 0x00 to an 
// internal 8-bit register located at Address 0xE7E3. Reinitialize 
// the registers, and write the last register in the queue three 
// times. Enable the write protection by writing 0xAD to an 
// internal 8-bit register located at Address 0xE7FE, followed 
// by a write of 0x80 to an internal 8-bit register located at 
// Address 0xE7E3. 
// Start the DSP by setting Run = 1. 
// 10. Read the energy registers xWATTHR, xVAHR, xFWATTHR, 
// and xFVARHR to erase their content and start energy 
// accumulation from a known state. 
// 11. Enable the CF1, CF2 and CF3 frequency converter outputs 
// by clearing bits 9, 10 and 11 (CF1DIS, CF2DIS, and 
// CF3DIS) to 0 in CFMODE register.  
// There is no obvious reason to stop the DSP if the ADE7880 is 
// maintained in PSM0 normal mode. All ADE7880 registers, 
// including ones located in the data memory RAM, can be 
// modified without stopping the DSP. However, to stop the DSP, 
// write 0x0000 into the Run register. To restart the DSP, follow 
// one of the following procedures:  
// • If the ADE7880 registers located in the data memory RAM 
// have not been modified, write 0x0001 into the Run register to 
// start the DSP.  
// • If the ADE7880 registers located in the data memory RAM 
// have to be modified, first execute a software or a hardware 
// reset, initialize all ADE7880 registers at desired values, enable 
// the write protection and then write 0x0001 into the Run 
// register to start the DSP.  
// As mentioned in the Power Management section, when the 
// ADE7880 switch out of PSM0 power mode, it is recommended to 
// stop the DSP by writing 0x0000 into the Run register (see Table 10 
// and Table 11 for the recommended actions when changing 
// power modes). 