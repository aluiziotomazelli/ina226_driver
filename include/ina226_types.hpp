#pragma once

#include <cstdint>

namespace ina226 {

static constexpr uint8_t DEFAULT_I2C_ADDR = 0x40;
static constexpr uint16_t MANUFACTURER_ID_VALUE = 0x5449; ///< "TI"
static constexpr uint16_t DIE_ID_VALUE = 0x2260;
static constexpr uint16_t RESET_BIT = (1 << 15); ///< RST: configuration reset bit (self-clearing)

/**
 * @enum Register
 * @brief INA226 Register Map addresses.
 */
enum class Register : uint8_t
{
    CONFIG = 0x00,          ///< Configuration register
    SHUNT_VOLTAGE = 0x01,   ///< Shunt voltage measurement
    BUS_VOLTAGE = 0x02,     ///< Bus voltage measurement
    POWER = 0x03,           ///< Power measurement
    CURRENT = 0x04,         ///< Current measurement
    CALIBRATION = 0x05,     ///< Calibration register
    MASK_ENABLE = 0x06,     ///< Interrupt mask and flag enable
    ALERT_LIMIT = 0x07,     ///< Alert limit register
    MANUFACTURER_ID = 0xFE, ///< Manufacturer ID (0x5449)
    DIE_ID = 0xFF           ///< Die ID (0x2260)
};

/**
 * @enum AveragingMode
 * @brief Number of samples averaged per conversion.
 */
enum class AveragingMode : uint16_t
{
    AVG_1 = 0b000,
    AVG_4 = 0b001,
    AVG_16 = 0b010,
    AVG_64 = 0b011,
    AVG_128 = 0b100,
    AVG_256 = 0b101,
    AVG_512 = 0b110,
    AVG_1024 = 0b111
};

constexpr uint16_t averaging_mode_to_count(AveragingMode mode)
{
    switch (mode) {
    case AveragingMode::AVG_1:
        return 1;
    case AveragingMode::AVG_4:
        return 4;
    case AveragingMode::AVG_16:
        return 16;
    case AveragingMode::AVG_64:
        return 64;
    case AveragingMode::AVG_128:
        return 128;
    case AveragingMode::AVG_256:
        return 256;
    case AveragingMode::AVG_512:
        return 512;
    case AveragingMode::AVG_1024:
        return 1024;
    default:
        return 1;
    }
}

/**
 * @enum ConversionTime
 * @brief Conversion time for VBUS or VSHUNT.
 */
enum class ConversionTime : uint16_t
{
    CT_140US = 0b000,
    CT_204US = 0b001,
    CT_332US = 0b010,
    CT_588US = 0b011,
    CT_1100US = 0b100,
    CT_2116US = 0b101,
    CT_4156US = 0b110,
    CT_8244US = 0b111
};

constexpr uint32_t conversion_time_to_us(ConversionTime ct)
{
    switch (ct) {
    case ConversionTime::CT_140US:
        return 140;
    case ConversionTime::CT_204US:
        return 204;
    case ConversionTime::CT_332US:
        return 332;
    case ConversionTime::CT_588US:
        return 588;
    case ConversionTime::CT_1100US:
        return 1100;
    case ConversionTime::CT_2116US:
        return 2116;
    case ConversionTime::CT_4156US:
        return 4156;
    case ConversionTime::CT_8244US:
        return 8244;
    default:
        return 1100;
    }
}

/**
 * @enum OperatingMode
 * @brief ADC Operating Mode.
 */
enum class OperatingMode : uint16_t
{
    POWER_DOWN = 0b000,
    SHUNT_TRIGGERED = 0b001,
    BUS_TRIGGERED = 0b010,
    SHUNT_AND_BUS_TRIGGERED = 0b011,
    ADC_OFF = 0b100,
    SHUNT_CONTINUOUS = 0b101,
    BUS_CONTINUOUS = 0b110,
    SHUNT_AND_BUS_CONTINUOUS = 0b111
};

/**
 * @enum AlertFlag
 * @brief Bitmasks for MASK_ENABLE register.
 */
enum class AlertFlag : uint16_t
{
    SHUNT_OVER_VOLTAGE = (1 << 15),        ///< SOL: Shunt Voltage Over-Voltage
    SHUNT_UNDER_VOLTAGE = (1 << 14),       ///< SUL: Shunt Voltage Under-Voltage
    BUS_OVER_VOLTAGE = (1 << 13),          ///< BOL: Bus Voltage Over-Voltage
    BUS_UNDER_VOLTAGE = (1 << 12),         ///< BUL: Bus Voltage Under-Voltage
    POWER_OVER_LIMIT = (1 << 11),          ///< POL: Power Over-Limit
    ALERT_ON_CONVERSION_READY = (1 << 10), ///< CNVR: assert Alert pin when a conversion completes
    ALERT_FUNCTION_FLAG = (1 << 4),        ///< AFF: Alert Function Flag
    CONVERSION_READY = (1 << 3),           ///< CVRF: conversion-ready status flag
    ALERT_POLARITY_HIGH = (1 << 1), ///< APOL: Alert polarity; 1 = inverted (active-high), 0 = normal (active-low)
    LATCH_ENABLE = (1 << 0)         ///< LEN: Alert latch enable
};

/**
 * @struct Ina226Config
 * @brief Configuration parameters for initializing INA226.
 */
struct Ina226Config
{
    uint8_t i2c_addr = DEFAULT_I2C_ADDR;
    float r_shunt_ohms = 0.1f;              ///< Shunt resistor value in Ohms (e.g. 0.1 for R100)
    float max_expected_current_a = 0.8192f; ///< Max expected current in Amperes
    AveragingMode avg_mode = AveragingMode::AVG_16;
    ConversionTime vbus_ct = ConversionTime::CT_1100US;
    ConversionTime vsh_ct = ConversionTime::CT_1100US;
    OperatingMode mode = OperatingMode::SHUNT_AND_BUS_CONTINUOUS;
};

} // namespace ina226
