#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/spi_master.h"
#include "ade7880_registers.h"

#ifdef __cplusplus
extern "C" {
#endif

// Initialize with config from menuconfig
esp_err_t ade7880_init(spi_device_handle_t *spi_handle);

// Reset the ADE7880
void ade7880_reset(void);

// Read/write registers
esp_err_t ade7880_write_register(spi_device_handle_t spi_handle, 
                               const ade7880_reg_t *reg, 
                               uint32_t value);
esp_err_t ade7880_read_register(spi_device_handle_t spi_handle, 
                              const ade7880_reg_t *reg, 
                              uint32_t *value);

// Start DSP operations
esp_err_t ade7880_start_dsp(spi_device_handle_t spi_handle);

// Read RMS values
void ade7880_read_rms_values(spi_device_handle_t spi_handle);

// Get version
esp_err_t ade7880_read_version(spi_device_handle_t spi_handle, uint8_t *version);

#ifdef __cplusplus
}
#endif