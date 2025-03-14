/*
 * ADE7880 SPI Test Application
 * Based on ESP-IDF LCD example
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"

static const char *TAG = "ADE7880_TEST";

// Using SPI2 in the example
#define ADE7880_SPI_HOST SPI2_HOST

// Pin definitions for SPI bus
#define ADE7880_PIN_NUM_SCLK 12
#define ADE7880_PIN_NUM_MOSI 10
#define ADE7880_PIN_NUM_MISO 11
#define ADE7880_PIN_NUM_CS 9
#define ADE7880_PIN_NUM_RESET 3 // IO3 for reset pin
#define ADE7880_PIN_NUM_IRQ0 14 // IRQ0 pin (already defined as IRQ)
#define ADE7880_PIN_NUM_IRQ1 13 // IO13 for IRQ1 pin

// ADE7880 Register Addresses
#define ADE7880_AIGAIN 0x4380  // Phase A current gain register
#define ADE7880_CONFIG0 0xE618 // Configuration register
#define ADE7880_VERSION 0xE707 // Chip version register
#define ADE7880_RUN 0xE228     // Run register
#define ADE7880_CONFIG2 0xEA00 // Configuration register 2 for locking the serial port
#define ADE7880_STATUS1 0xE503 // STATUS1 register
#define ADE7880_STATUS0 0xE502 // STATUS0 register

// SPI settings for ADE7880
#define ADE7880_SPI_SPEED 1000000 // 1MHz - adjust as needed
#define ADE7880_SPI_MODE 0        // CPOL=0, CPHA=0

// Function prototypes
static void IRAM_ATTR ade7880_irq0_handler(void *arg);
static void IRAM_ATTR ade7880_irq1_handler(void *arg);
esp_err_t ade7880_gpio_init(void);
esp_err_t ade7880_reset(void);
esp_err_t ade7880_init(void);
uint32_t ade7880_read_register(uint16_t reg_addr);
esp_err_t ade7880_write_register(uint16_t reg_addr, uint32_t value);
esp_err_t ade7880_lock_spi_interface(void);
esp_err_t ade7880_init_after_reset(void);

// SPI device handle
spi_device_handle_t ade7880_spi = NULL;

// Initialize GPIO pins for ADE7880
esp_err_t ade7880_gpio_init(void)
{
    esp_err_t ret;

    // Configure reset pin as output
    gpio_config_t io_conf_reset = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << ADE7880_PIN_NUM_RESET),
        .pull_down_en = 0,
        .pull_up_en = 0,
    };
    ret = gpio_config(&io_conf_reset);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to configure reset pin: %s", esp_err_to_name(ret));
        return ret;
    }

    // Configure interrupt pins as inputs with interrupts
    gpio_config_t io_conf_int = {
        .intr_type = GPIO_INTR_NEGEDGE, // Trigger on falling edge
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = ((1ULL << ADE7880_PIN_NUM_IRQ0) | (1ULL << ADE7880_PIN_NUM_IRQ1)),
        .pull_up_en = 1, // Enable pull-up
    };
    ret = gpio_config(&io_conf_int);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to configure interrupt pins: %s", esp_err_to_name(ret));
        return ret;
    }

    // Install GPIO ISR service
    ret = gpio_install_isr_service(0);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE)
    { // ESP_ERR_INVALID_STATE means already installed
        ESP_LOGE(TAG, "Failed to install GPIO ISR service: %s", esp_err_to_name(ret));
        return ret;
    }

    // Add ISR handlers for each interrupt pin
    ret = gpio_isr_handler_add(ADE7880_PIN_NUM_IRQ0, ade7880_irq0_handler, NULL);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to add IRQ0 handler: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = gpio_isr_handler_add(ADE7880_PIN_NUM_IRQ1, ade7880_irq1_handler, NULL);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to add IRQ1 handler: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "ADE7880 GPIO initialization complete");
    return ESP_OK;
}

// Reset the ADE7880 via IO3 and configure SPI communication
esp_err_t ade7880_reset(void)
{
    esp_err_t ret;
    ESP_LOGI(TAG, "Resetting ADE7880");

    ESP_LOGD(TAG, "Pulling reset pin low");
    // Reset sequence: Pull reset pin low for at least 10µs
    gpio_set_level(ADE7880_PIN_NUM_RESET, 0);
    vTaskDelay(pdMS_TO_TICKS(10)); // 10ms delay for safety (much longer than required)
    ESP_LOGD(TAG, "Pulling reset pin high");
    gpio_set_level(ADE7880_PIN_NUM_RESET, 1);
    vTaskDelay(pdMS_TO_TICKS(1)); // Wait just a bit for device to stabilize after reset

    // After reset, toggle SS/HSA pin (CS pin) three times to select SPI interface
    // This must be done before any other SPI communication
    ESP_LOGD(TAG, "Initializing SPI interface selection");
    for (int i = 0; i < 3; i++)
    {
        ESP_LOGD(TAG, "Toggling CS pin %d", i + 1);
        gpio_set_level(ADE7880_PIN_NUM_CS, 1);
        vTaskDelay(pdMS_TO_TICKS(1)); // Small delay
        ESP_LOGD(TAG, "Toggling CS pin %d", i + 1);
        gpio_set_level(ADE7880_PIN_NUM_CS, 0);
        vTaskDelay(pdMS_TO_TICKS(1)); // Small delay
        ESP_LOGD(TAG, "Toggling CS pin %d", i + 1);
        gpio_set_level(ADE7880_PIN_NUM_CS, 1);
        vTaskDelay(pdMS_TO_TICKS(1)); // Small delay
    }
    ESP_LOGD(TAG, "CS pin toggled three times");
    vTaskDelay(pdMS_TO_TICKS(1)); // Allow time to process

    // Wait for RSTDONE interrupt on IRQ1 (signaling end of initialization)
    ESP_LOGI(TAG, "Waiting for RSTDONE interrupt (IRQ1 low)...");
    int timeout_ms = 50; // 50ms timeout
    while (gpio_get_level(ADE7880_PIN_NUM_IRQ1) == 1 && timeout_ms > 0)
    {
        vTaskDelay(1); // 1ms delay
        timeout_ms--;
    }

    if (timeout_ms <= 0)
    {
        ESP_LOGW(TAG, "Timeout waiting for RSTDONE interrupt");
    }
    else
    {
        ESP_LOGI(TAG, "RSTDONE interrupt detected");
    }

    ESP_LOGI(TAG, "ADE7880 reset complete and SPI interface selected");
    return ESP_OK;
}

// Lock the SPI interface
esp_err_t ade7880_lock_spi_interface(void)
{
    ESP_LOGI(TAG, "Locking SPI interface");

    // Any write to CONFIG2 locks the SPI interface
    esp_err_t ret = ade7880_write_register(ADE7880_CONFIG2, 0x0000);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to lock SPI interface");
        return ret;
    }

    ESP_LOGI(TAG, "SPI interface locked successfully");
    return ESP_OK;
}

// IRQ0 interrupt handler
static void IRAM_ATTR ade7880_irq0_handler(void *arg)
{
    // This is an ISR, keep it short and simple
    // In a real application, you would set a flag or notify a task
    static uint32_t last_int_time = 0;
    uint32_t now = xTaskGetTickCountFromISR();

    // Simple debounce
    if (now - last_int_time > pdMS_TO_TICKS(50))
    {
        // In a real application, you could set a flag or send a notification
        // to process the interrupt in the main task context
    }
    last_int_time = now;
}

// IRQ1 interrupt handler
static void IRAM_ATTR ade7880_irq1_handler(void *arg)
{
    // This is an ISR, keep it short and simple
    // In a real application, you would set a flag or notify a task
    static uint32_t last_int_time = 0;
    uint32_t now = xTaskGetTickCountFromISR();

    // Simple debounce
    if (now - last_int_time > pdMS_TO_TICKS(50))
    {
        // Handle IRQ1 interrupt
        // Flag handling will be done in main task
    }
    last_int_time = now;
}

// Initialize SPI communication with ADE7880
esp_err_t ade7880_init(void)
{
    esp_err_t ret;

    // First initialize GPIO pins
    ret = ade7880_gpio_init();
    if (ret != ESP_OK)
    {
        return ret;
    }

    // Reset the ADE7880 and configure for SPI communication
    ret = ade7880_reset();
    if (ret != ESP_OK)
    {
        return ret;
    }

    ESP_LOGI(TAG, "Initializing SPI bus for ADE7880");
    spi_bus_config_t buscfg = {
        .sclk_io_num = ADE7880_PIN_NUM_SCLK,
        .mosi_io_num = ADE7880_PIN_NUM_MOSI,
        .miso_io_num = ADE7880_PIN_NUM_MISO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32, // We only need small transfers for register access
    };

    // Initialize the SPI bus
    ret = spi_bus_initialize(ADE7880_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
        return ret;
    }

    // Configure SPI device
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = ADE7880_SPI_SPEED,
        .mode = ADE7880_SPI_MODE,
        .spics_io_num = ADE7880_PIN_NUM_CS,
        .queue_size = 7,
        .command_bits = 0,
        .address_bits = 0,
    };

    // Attach device to the SPI bus
    ret = spi_bus_add_device(ADE7880_SPI_HOST, &devcfg, &ade7880_spi);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to add SPI device: %s", esp_err_to_name(ret));
        return ret;
    }

    // Complete ADE7880 initialization after SPI setup
    ret = ade7880_init_after_reset();
    if (ret != ESP_OK)
    {
        return ret;
    }

    // Lock the SPI interface to prevent accidental switching
    ret = ade7880_lock_spi_interface();
    if (ret != ESP_OK)
    {
        return ret;
    }

    ESP_LOGI(TAG, "ADE7880 SPI initialization complete");
    return ESP_OK;
}

// Read from a register in ADE7880
uint32_t ade7880_read_register(uint16_t reg_addr)
{
    uint8_t tx_data[4] = {0};
    uint8_t rx_data[4] = {0};
    uint32_t result = 0;

    // For ADE7880 SPI read protocol:
    // First byte: command byte with MSB=1 (for read)
    // Second byte: register address MSB
    // Third byte: register address LSB
    tx_data[0] = 0x01;                   // Read command (MSB = 0, bit 6 = 1 for read)
    tx_data[1] = (reg_addr >> 8) & 0xFF; // Register address MSB
    tx_data[2] = reg_addr & 0xFF;        // Register address LSB

    spi_transaction_t t = {
        .length = 32, // Total bits to transfer (command + address + data)
        .tx_buffer = tx_data,
        .rx_buffer = rx_data,
    };

    esp_err_t ret = spi_device_polling_transmit(ade7880_spi, &t);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "SPI read transaction failed: %s", esp_err_to_name(ret));
        return 0;
    }

    // Combine bytes into 32-bit result
    result = (rx_data[0] << 24) | (rx_data[1] << 16) | (rx_data[2] << 8) | rx_data[3];
    return result;
}

// Write to a register in ADE7880
esp_err_t ade7880_write_register(uint16_t reg_addr, uint32_t value)
{
    uint8_t tx_data[7] = {0};

    // For ADE7880 SPI write protocol:
    // First byte: command byte with MSB=0 (for write)
    // Second byte: register address MSB
    // Third byte: register address LSB
    // Fourth-Seventh bytes: data to write (32-bit)
    tx_data[0] = 0x00;                   // Write command
    tx_data[1] = (reg_addr >> 8) & 0xFF; // Register address MSB
    tx_data[2] = reg_addr & 0xFF;        // Register address LSB
    tx_data[3] = (value >> 24) & 0xFF;   // Data MSB
    tx_data[4] = (value >> 16) & 0xFF;
    tx_data[5] = (value >> 8) & 0xFF;
    tx_data[6] = value & 0xFF; // Data LSB

    spi_transaction_t t = {
        .length = 7 * 8, // Total bits to transfer
        .tx_buffer = tx_data,
    };

    esp_err_t ret = spi_device_polling_transmit(ade7880_spi, &t);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "SPI write transaction failed: %s", esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}

void test_ade7880_connection(void)
{
    // Read the version register to verify communication
    uint32_t version = ade7880_read_register(ADE7880_VERSION);
    ESP_LOGI(TAG, "ADE7880 Version Register: 0x%08lx", version);

    if (version == 0 || version == 0xFFFFFFFF)
    {
        ESP_LOGE(TAG, "Failed to read ADE7880 version register. Check connections and power.");
    }
    else
    {
        ESP_LOGI(TAG, "Successfully communicated with ADE7880!");

        // Test reading some common registers
        uint32_t config = ade7880_read_register(ADE7880_CONFIG0);
        ESP_LOGI(TAG, "CONFIG0 Register: 0x%08lx", config);

        // Test writing to a register (AIGAIN)
        ESP_LOGI(TAG, "Writing test value to AIGAIN register");
        uint32_t test_value = 0x400000; // Example gain value
        ade7880_write_register(ADE7880_AIGAIN, test_value);

        // Verify by reading back
        uint32_t read_back = ade7880_read_register(ADE7880_AIGAIN);
        ESP_LOGI(TAG, "AIGAIN Register (after write): 0x%08lx", read_back);

        if (read_back == test_value)
        {
            ESP_LOGI(TAG, "Write/read test successful!");
        }
        else
        {
            ESP_LOGI(TAG, "Write/read test failed. Expected: 0x%08lx, Got: 0x%08lx",
                     test_value, read_back);
        }
    }
}

void app_main(void)
{
    // Set log level to Debug for all tags
    esp_log_level_set("*", ESP_LOG_VERBOSE);
    esp_log_level_set(TAG, ESP_LOG_VERBOSE);

    ESP_LOGI(TAG, "Starting ADE7880 SPI test application");

    // Initialize SPI for ADE7880
    esp_err_t ret = ade7880_init();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "ADE7880 initialization failed");
        return;
    }

    // Test connection and basic functionality
    test_ade7880_connection();

    ESP_LOGI(TAG, "ADE7880 SPI test completed");
}

// Add this function to initialize the ADE7880 after serial interface is established
esp_err_t ade7880_init_after_reset(void)
{
    esp_err_t ret;

    // Clear RSTDONE bit in STATUS1 register (write 1 to clear)
    ret = ade7880_write_register(ADE7880_STATUS1, (1 << 15));
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to clear RSTDONE bit");
        return ret;
    }

    // Clear all other status flags as good practice
    ret = ade7880_write_register(ADE7880_STATUS0, 0xFFFFFFFF);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to clear STATUS0 register");
        return ret;
    }

    // At this point, all registers should be initialized before starting the DSP

    // Put ADE7880 into run mode
    ret = ade7880_write_register(ADE7880_RUN, 0x0001);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to put ADE7880 into run mode");
        return ret;
    }

    return ESP_OK;
}