#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "esp_log.h"
#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "ade7880.h"

static const char *TAG = "ade7880";

void ade7880_reset(void)
{
    // Configure RESET pin as output
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << CONFIG_ENERGY_ADE7880_RESET_PIN),
        .pull_down_en = 0,
        .pull_up_en = 0
    };
    gpio_config(&io_conf);
    
    // Reset sequence
    ESP_LOGI(TAG, "Resetting ADE7880");
    gpio_set_level(CONFIG_ENERGY_ADE7880_RESET_PIN, 0);    // Assert reset (active low)
    vTaskDelay(10 / portTICK_PERIOD_MS);     // Hold for 10ms
    gpio_set_level(CONFIG_ENERGY_ADE7880_RESET_PIN, 1);    // Release reset
    vTaskDelay(100 / portTICK_PERIOD_MS);    // Wait for device to initialize
}

static void toggle_cs_pin(void)
{
    ESP_LOGI(TAG, "Toggling CS pin");
    for (int i = 0; i < 3; i++) {
        gpio_set_level(CONFIG_ENERGY_ADE7880_CS_PIN, 1);   // CS high
        vTaskDelay(10 / portTICK_PERIOD_MS);
        gpio_set_level(CONFIG_ENERGY_ADE7880_CS_PIN, 0);   // CS low
        vTaskDelay(10 / portTICK_PERIOD_MS);
        gpio_set_level(CONFIG_ENERGY_ADE7880_CS_PIN, 1);   // CS high
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

static esp_err_t spi_init(spi_device_handle_t *spi_handle)
{
    ESP_LOGI(TAG, "Initializing SPI bus");
    
    // Initialize SPI bus
    spi_bus_config_t bus_config = {
        .mosi_io_num = CONFIG_ENERGY_ADE7880_MOSI_PIN,
        .miso_io_num = CONFIG_ENERGY_ADE7880_MISO_PIN,
        .sclk_io_num = CONFIG_ENERGY_ADE7880_SCLK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32,
    };
    
    esp_err_t ret = spi_bus_initialize(CONFIG_ENERGY_ADE7880_SPI_HOST, &bus_config, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Configure CS pin as GPIO
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << CONFIG_ENERGY_ADE7880_CS_PIN),
        .pull_down_en = 0,
        .pull_up_en = 0
    };
    gpio_config(&io_conf);
    gpio_set_level(CONFIG_ENERGY_ADE7880_CS_PIN, 1);  // Start with CS high
    
    // Configure SPI device
    spi_device_interface_config_t dev_config = {
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .mode = 0,                   // SPI mode 0
        .clock_speed_hz = CONFIG_ENERGY_ADE7880_SPI_CLOCK_SPEED_HZ,
        .spics_io_num = -1,          // We'll manage CS pin manually
        .queue_size = 1,
    };
    
    ret = spi_bus_add_device(CONFIG_ENERGY_ADE7880_SPI_HOST, &dev_config, spi_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add SPI device: %s", esp_err_to_name(ret));
        return ret;
    }
    
    return ESP_OK;
}

esp_err_t ade7880_write_register(spi_device_handle_t spi_handle, 
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
    
    gpio_set_level(CONFIG_ENERGY_ADE7880_CS_PIN, 0);
    esp_err_t ret = spi_device_polling_transmit(spi_handle, &t);
    gpio_set_level(CONFIG_ENERGY_ADE7880_CS_PIN, 1);
    
    return ret;
}

esp_err_t ade7880_read_register(spi_device_handle_t spi_handle, 
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
    
    gpio_set_level(CONFIG_ENERGY_ADE7880_CS_PIN, 0);
    esp_err_t ret = spi_device_polling_transmit(spi_handle, &t);
    gpio_set_level(CONFIG_ENERGY_ADE7880_CS_PIN, 1);
    
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

esp_err_t ade7880_read_version(spi_device_handle_t spi_handle, uint8_t *version)
{
    uint32_t value = 0;
    esp_err_t ret = ade7880_read_register(spi_handle, &ADE7880_VERSION_REG, &value);
    if (ret == ESP_OK) {
        *version = (uint8_t)(value & 0xFF);
    }
    return ret;
}

void ade7880_read_rms_values(spi_device_handle_t spi_handle)
{
    // Use the existing read_rms_values implementation
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

esp_err_t ade7880_start_dsp(spi_device_handle_t spi_handle)
{
    ESP_LOGI(TAG, "Starting ADE7880 DSP");
    // Write 1 to RUN register to start DSP
    return ade7880_write_register(spi_handle, &ADE7880_RUN_REG, 1);
}

esp_err_t ade7880_init(spi_device_handle_t *spi_handle)
{
    esp_err_t ret;
    
    // Step 1: Reset the device
    ade7880_reset();
    
    // Step 2: Initialize SPI
    ret = spi_init(spi_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI initialization failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Step 3: Toggle CS pin to ensure SPI interface is selected
    toggle_cs_pin();
    
    // Step 4: Verify device by reading version register
    uint8_t version;
    ret = ade7880_read_version(*spi_handle, &version);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read version register: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "ADE7880 Version: 0x%02x", version);
    
    // Step 5: Test reading and writing to registers
    ret = test_register_write_read(*spi_handle);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Register read/write tests failed: %s", esp_err_to_name(ret));
        // Continue anyway, might still work
    }
    
    // Step 6: Verify default register values
    test_default_registers(*spi_handle);
    
    // Step 7: Start DSP
    ret = ade7880_start_dsp(*spi_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start DSP: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "ADE7880 initialization complete");
    return ESP_OK;
}