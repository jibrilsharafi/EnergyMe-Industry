/**
 * @file ade7880_registers.h
 * @brief Register definitions for ADE7880 energy metering IC
 *
 * This file contains all register addresses and definitions for the ADE7880
 * polyphase multifunction energy metering IC.
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>

/* ADE7880 Register Information Structure */
typedef struct {
    uint16_t address;         // Register address
    uint8_t comm_length;      // Communication bit length (8, 16, or 32)
    bool writable;            // Whether the register is writable
    uint32_t default_value;   // Default value according to datasheet
    const char* name;         // Register name for logging purposes
} ade7880_reg_t;

/* Register Definitions */

// DSP Control register
static const ade7880_reg_t ADE7880_RUN_REG = {
    .address = 0xE228,
    .comm_length = 16,
    .writable = true,
    .default_value = 0x000000,
    .name = "RUN_REG"
};

// Version register
static const ade7880_reg_t ADE7880_VERSION_REG = {
    .address = 0xE707,
    .comm_length = 8,
    .writable = false,
    .default_value = 0x000000,
    .name = "VERSION_REG"
};

// DSP RAM Gain registers (need byte shifting)
static const ade7880_reg_t ADE7880_AIGAIN = {
    .address = 0x4380,
    .comm_length = 32,
    .writable = true,
    .default_value = 0x000000,
    .name = "AIGAIN"
};

static const ade7880_reg_t ADE7880_AVGAIN = {
    .address = 0x4381,
    .comm_length = 32,
    .writable = true,
    .default_value = 0x000000,
    .name = "AVGAIN"
};

static const ade7880_reg_t ADE7880_BIGAIN = {
    .address = 0x4382,
    .comm_length = 32,
    .writable = true,
    .default_value = 0x000000,
    .name = "BIGAIN"
};

static const ade7880_reg_t ADE7880_BVGAIN = {
    .address = 0x4383,
    .comm_length = 32,
    .writable = true,
    .default_value = 0x000000,
    .name = "BVGAIN"
};

static const ade7880_reg_t ADE7880_CIGAIN = {
    .address = 0x4384,
    .comm_length = 32,
    .writable = true,
    .default_value = 0x000000,
    .name = "CIGAIN"
};

static const ade7880_reg_t ADE7880_CVGAIN = {
    .address = 0x4385,
    .comm_length = 32,
    .writable = true,
    .default_value = 0x000000,
    .name = "CVGAIN"
};

// RMS registers (read-only)
static const ade7880_reg_t ADE7880_AIRMS = {
    .address = 0x43C0,
    .comm_length = 32,
    .writable = false,
    .default_value = 0x000000,
    .name = "AIRMS"
};

static const ade7880_reg_t ADE7880_AVRMS = {
    .address = 0x43C1,
    .comm_length = 32,
    .writable = false,
    .default_value = 0x000000,
    .name = "AVRMS"
};

static const ade7880_reg_t ADE7880_BIRMS = {
    .address = 0x43C2,
    .comm_length = 32,
    .writable = false,
    .default_value = 0x000000,
    .name = "BIRMS"
};

static const ade7880_reg_t ADE7880_BVRMS = {
    .address = 0x43C3,
    .comm_length = 32,
    .writable = false,
    .default_value = 0x000000,
    .name = "BVRMS"
};

static const ade7880_reg_t ADE7880_CIRMS = {
    .address = 0x43C4,
    .comm_length = 32,
    .writable = false,
    .default_value = 0x000000,
    .name = "CIRMS"
};

static const ade7880_reg_t ADE7880_CVRMS = {
    .address = 0x43C5,
    .comm_length = 32,
    .writable = false,
    .default_value = 0x000000,
    .name = "CVRMS"
};

// Configuration and Control Registers
static const ade7880_reg_t ADE7880_LINECYC = {
    .address = 0xE60C,
    .comm_length = 16,
    .writable = true,
    .default_value = 0xFFFF,
    .name = "LINECYC"
};

static const ade7880_reg_t ADE7880_ZXTOUT = {
    .address = 0xE60D,
    .comm_length = 16,
    .writable = true,
    .default_value = 0xFFFF,
    .name = "ZXTOUT"
};

static const ade7880_reg_t ADE7880_COMPMODE = {
    .address = 0xE60E,
    .comm_length = 16,
    .writable = true,
    .default_value = 0x01FF,
    .name = "COMPMODE"
};

static const ade7880_reg_t ADE7880_GAIN = {
    .address = 0xE60F,
    .comm_length = 16,
    .writable = true,
    .default_value = 0x0000,
    .name = "GAIN"
};

static const ade7880_reg_t ADE7880_CFMODE = {
    .address = 0xE610,
    .comm_length = 16,
    .writable = true,
    .default_value = 0x0EA0,
    .name = "CFMODE"
};

static const ade7880_reg_t ADE7880_CF1DEN = {
    .address = 0xE611,
    .comm_length = 16,
    .writable = true,
    .default_value = 0x0000,
    .name = "CF1DEN"
};

static const ade7880_reg_t ADE7880_CF2DEN = {
    .address = 0xE612,
    .comm_length = 16,
    .writable = true,
    .default_value = 0x0000,
    .name = "CF2DEN"
};

static const ade7880_reg_t ADE7880_CF3DEN = {
    .address = 0xE613,
    .comm_length = 16,
    .writable = true,
    .default_value = 0x0000,
    .name = "CF3DEN"
};

static const ade7880_reg_t ADE7880_APHCAL = {
    .address = 0xE614,
    .comm_length = 16,
    .writable = true,
    .default_value = 0x0000,
    .name = "APHCAL"
};

static const ade7880_reg_t ADE7880_BPHCAL = {
    .address = 0xE615,
    .comm_length = 16,
    .writable = true,
    .default_value = 0x0000,
    .name = "BPHCAL"
};

static const ade7880_reg_t ADE7880_CPHCAL = {
    .address = 0xE616,
    .comm_length = 16,
    .writable = true,
    .default_value = 0x0000,
    .name = "CPHCAL"
};

static const ade7880_reg_t ADE7880_PHSIGN = {
    .address = 0xE617,
    .comm_length = 16,
    .writable = false,
    .default_value = 0x0000,
    .name = "PHSIGN"
};

static const ade7880_reg_t ADE7880_CONFIG = {
    .address = 0xE618,
    .comm_length = 16,
    .writable = true,
    .default_value = 0x0002,
    .name = "CONFIG"
};

static const ade7880_reg_t ADE7880_MMODE = {
    .address = 0xE700,
    .comm_length = 8,
    .writable = true,
    .default_value = 0x1C,
    .name = "MMODE"
};

static const ade7880_reg_t ADE7880_ACCMODE = {
    .address = 0xE701,
    .comm_length = 8,
    .writable = true,
    .default_value = 0x80,
    .name = "ACCMODE"
};

static const ade7880_reg_t ADE7880_LCYCMODE = {
    .address = 0xE702,
    .comm_length = 8,
    .writable = true,
    .default_value = 0x78,
    .name = "LCYCMODE"
};

static const ade7880_reg_t ADE7880_PEAKCYC = {
    .address = 0xE703,
    .comm_length = 8,
    .writable = true,
    .default_value = 0x00,
    .name = "PEAKCYC"
};

static const ade7880_reg_t ADE7880_SAGCYC = {
    .address = 0xE704,
    .comm_length = 8,
    .writable = true,
    .default_value = 0x00,
    .name = "SAGCYC"
};

static const ade7880_reg_t ADE7880_CFCYC = {
    .address = 0xE705,
    .comm_length = 8,
    .writable = true,
    .default_value = 0x01,
    .name = "CFCYC"
};

static const ade7880_reg_t ADE7880_HSDC_CFG = {
    .address = 0xE706,
    .comm_length = 8,
    .writable = true,
    .default_value = 0x00,
    .name = "HSDC_CFG"
};