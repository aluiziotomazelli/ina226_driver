#pragma once

#include "esp_err.h"
#include "interfaces/i_hal_i2c.hpp"
#include "ina226_types.hpp"

namespace ina226 {

/**
 * @interface IIna226Driver
 * @brief Pure virtual interface for the INA226 Bi-directional Current/Power Monitor driver.
 *
 * Designed for Dependency Injection and Unit Testing (Mocking). All methods
 * perform blocking I2C transfers and report status via esp_err_t; out-parameters
 * are only valid when the call returns ESP_OK.
 */
class IIna226Driver
{
public:
    virtual ~IIna226Driver() = default;

    /**
     * @brief Adds the INA226 device to an initialized I2C master bus and performs
     *        hardware initialization (chip ID verification + configuration write).
     *
     * Entry point for normal use: internally calls init() after registering the
     * device on the bus.
     * @param[in] bus_handle Initialized I2C master bus handle.
     * @return ESP_OK on success, or ESP_ERR_* on I2C failure / ID mismatch.
     */
    virtual esp_err_t init(i2c_master_bus_handle_t bus_handle) = 0;

    /**
     * @brief Verifies the INA226 chip IDs and writes the current configuration.
     *
     * Requires the device to be registered on a bus first (see init(bus_handle)).
     * @return ESP_OK on success, ESP_ERR_INVALID_STATE if no device is registered,
     *         ESP_ERR_NOT_FOUND if the manufacturer ID does not match,
     *         ESP_ERR_INVALID_RESPONSE if the die ID does not match,
     *         or ESP_ERR_* on I2C failure.
     */
    virtual esp_err_t init() = 0;

    /**
     * @brief Performs a soft reset of the INA226 registers.
     *
     * Polls the self-clearing RST bit (datasheet SBOS448B) until it reads back 0,
     * then re-applies the active configuration and calibration.
     * @return ESP_OK on success, ESP_ERR_TIMEOUT if the RST bit never clears,
     *         or ESP_ERR_* on I2C failure.
     */
    virtual esp_err_t reset() = 0;

    /**
     * @brief Reads the shunt voltage in microvolts (uV).
     * @param[out] out_uv Measured shunt voltage; signed (negative = reverse current direction).
     * @return ESP_OK on success, or ESP_ERR_* on I2C failure.
     */
    virtual esp_err_t read_shunt_voltage_uv(int32_t& out_uv) = 0;

    /**
     * @brief Reads the bus voltage in millivolts (mV).
     * @param[out] out_mv Measured bus voltage; unsigned (INA226 bus range is 0-36 V).
     * @return ESP_OK on success, or ESP_ERR_* on I2C failure.
     */
    virtual esp_err_t read_bus_voltage_mv(uint16_t& out_mv) = 0;

    /**
     * @brief Reads the current in milliamperes (mA), as computed by the INA226.
     *
     * Reads the CURRENT register (04h) and scales by the Current_LSB derived at
     * calibration time. Requires a valid calibration to have been applied (see
     * calibrate() / Ina226Config).
     * @param[out] out_ma Measured current in mA; signed (bidirectional measurement).
     * @return ESP_OK on success, or ESP_ERR_* on I2C failure.
     */
    virtual esp_err_t read_current_ma(float& out_ma) = 0;

    /**
     * @brief Reads the power in milliwatts (mW), as computed by the INA226.
     * @param[out] out_mw Measured power in mW.
     * @return ESP_OK on success, or ESP_ERR_* on I2C failure.
     */
    virtual esp_err_t read_power_mw(float& out_mw) = 0;

    /**
     * @brief Applies a new configuration and recalibrates the device.
     *
     * Writes the CONFIG register from the new parameters and recomputes the
     * CALIBRATION register from r_shunt_ohms / max_expected_current_a.
     * @param[in] new_config Configuration to apply.
     * @return ESP_OK on success, ESP_ERR_INVALID_ARG if the computed CAL value
     *         exceeds the 15-bit CALIBRATION register, or ESP_ERR_* on I2C failure.
     */
    virtual esp_err_t set_config(const Ina226Config& new_config) = 0;

    /**
     * @brief Recomputes and writes the CALIBRATION register.
     *
     * Current_LSB = max_expected_current_a / 32768 and
     * CAL = 0.00512 / (Current_LSB * r_shunt_ohms). CAL must fit in the 15
     * writeable bits of the CALIBRATION register (max 0x7FFF, datasheet SBOS448B).
     * @param[in] r_shunt_ohms Shunt resistance in Ohms (must be > 0).
     * @param[in] max_expected_current_a Maximum expected current in Amperes (must be > 0).
     * @return ESP_OK on success, ESP_ERR_INVALID_ARG if any argument is <= 0 or
     *         the computed CAL exceeds 0x7FFF, or ESP_ERR_* on I2C failure.
     */
    virtual esp_err_t calibrate(float r_shunt_ohms, float max_expected_current_a) = 0;

    /**
     * @brief Configures the alert threshold and the MASK_ENABLE bits.
     *
     * Writes ALERT_LIMIT (07h) with alert_limit, then MASK_ENABLE (06h) with the
     * given AlertFlag bitmask. The ALERT pin is open-drain and active-low by
     * default (APOL = 0); it asserts while any enabled alert condition is true
     * (or stays asserted until MASK_ENABLE is read when LEN = 1).
     *
     * Note: when ALERT_ON_CONVERSION_READY (CNVR) is enabled, the ALERT pin is
     * asserted on every conversion completion and only deasserts after MASK_ENABLE
     * is read. Acknowledge the alert with read_alert_flags() to re-arm the pin.
     * @param[in] alert_mask Bitmask of AlertFlag values (e.g. SHUNT_OVER_VOLTAGE).
     * @param[in] alert_limit Raw threshold value for the ALERT_LIMIT register.
     * @return ESP_OK on success, or ESP_ERR_* on I2C failure.
     */
    virtual esp_err_t configure_alert(uint16_t alert_mask, uint16_t alert_limit) = 0;

    /**
     * @brief Reads and clears the alert flags in the MASK_ENABLE register.
     *
     * Reads MASK_ENABLE (06h) and returns the raw 16-bit value. Because AFF
     * (bit 4) and CVRF (bit 3) are read-to-clear (datasheet SBOS448B), this call
     * also deasserts the ALERT pin and re-arms the next alert condition. This is
     * required when using ALERT_ON_CONVERSION_READY (CNVR): the pin stays
     * asserted until MASK_ENABLE is read.
     * @param[out] out_flags Raw MASK_ENABLE value (AlertFlag bitmask) after the read.
     * @return ESP_OK on success, or ESP_ERR_* on I2C failure.
     */
    virtual esp_err_t read_alert_flags(uint16_t& out_flags) = 0;

    /**
     * @brief Checks the conversion-ready flag (CVRF) in the MASK_ENABLE register.
     *
     * CVRF (bit 3) is set when a new conversion completes and is cleared when
     * MASK_ENABLE is read (datasheet SBOS448B). Internally uses read_alert_flags(),
     * so it also clears the flags and deasserts the ALERT pin.
     * @param[out] out_ready Set to true when a new measurement is available.
     * @return ESP_OK on success, or ESP_ERR_* on I2C failure.
     */
    virtual esp_err_t is_conversion_ready(bool& out_ready) = 0;

    /**
     * @brief Returns the currently active configuration.
     * @return Reference to the active Ina226Config.
     */
    virtual const Ina226Config& get_config() const = 0;
};

} // namespace ina226
