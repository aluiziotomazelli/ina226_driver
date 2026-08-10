#pragma once

#include "esp_err.h"
#include "interfaces/i_hal_i2c.hpp"
#include "ina226_types.hpp"

namespace ina226 {

/**
 * @interface IIna226Driver
 * @brief Pure virtual interface for INA226 Bi-directional Current/Power Monitor driver.
 *
 * Designed for Dependency Injection and Unit Testing (Mocking).
 */
class IIna226Driver
{
public:
    virtual ~IIna226Driver() = default;

    /**
     * @brief Registers the INA226 device on the provided I2C master bus.
     *        Must be called before init().
     * @param bus_handle Initialized I2C master bus handle.
     * @return ESP_OK on success, or ESP_ERR_* on failure.
     */
    virtual esp_err_t init(i2c_master_bus_handle_t bus_handle) = 0;

    /**
     * @brief Initializes the INA226 hardware, verifies chip ID and writes configuration.
     * @return ESP_OK on success, or ESP_ERR_* on I2C failure / ID mismatch.
     */
    virtual esp_err_t init() = 0;

    /**
     * @brief Performs a soft reset of the INA226 registers.
     * @return ESP_OK on success.
     */
    virtual esp_err_t reset() = 0;

    /**
     * @brief Reads the raw shunt voltage in microvolts (uV).
     * @param[out] out_uv Reference to store measured shunt voltage (signed uV).
     * @return ESP_OK on success.
     */
    virtual esp_err_t read_shunt_voltage_uv(int32_t& out_uv) = 0;

    /**
     * @brief Reads the bus voltage in millivolts (mV).
     * @param[out] out_mv Reference to store measured bus voltage (unsigned mV).
     * @return ESP_OK on success.
     */
    virtual esp_err_t read_bus_voltage_mv(uint16_t& out_mv) = 0;

    /**
     * @brief Reads calculated current in milliamperes (mA) using current calibration.
     * @param[out] out_ma Reference to store measured current in mA.
     * @return ESP_OK on success.
     */
    virtual esp_err_t read_current_ma(float& out_ma) = 0;

    /**
     * @brief Reads calculated power in milliwatts (mW).
     * @param[out] out_mw Reference to store measured power in mW.
     * @return ESP_OK on success.
     */
    virtual esp_err_t read_power_mw(float& out_mw) = 0;

    /**
     * @brief Calibrates the current calculation register.
     * @param r_shunt_ohms Shunt resistance value in Ohms.
     * @param max_expected_current_a Maximum expected current in Amperes.
     * @return ESP_OK on success.
     */
    virtual esp_err_t calibrate(float r_shunt_ohms, float max_expected_current_a) = 0;

    /**
     * @brief Configures ALERT pin mask and limit register.
     * @param alert_mask Bitmask of AlertFlag values.
     * @param alert_limit Alert threshold value.
     * @return ESP_OK on success.
     */
    virtual esp_err_t configure_alert(uint16_t alert_mask, uint16_t alert_limit) = 0;

    /**
     * @brief Checks if conversion ready flag (CVRF) is set in MASK_ENABLE register.
     * @param[out] out_ready Set to true if new measurement conversion is ready.
     * @return ESP_OK on success.
     */
    virtual esp_err_t is_conversion_ready(bool& out_ready) = 0;

    /**
     * @brief Gets current active configuration.
     */
    virtual const Ina226Config& get_config() const = 0;
};

} // namespace ina226
