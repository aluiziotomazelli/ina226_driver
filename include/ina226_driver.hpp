#pragma once

#include "interfaces/i_ina226_driver.hpp"
#include "interfaces/i_hal_i2c.hpp"

namespace ina226 {

/**
 * @class Ina226Driver
 * @brief Concrete implementation of IIna226Driver interface using idf_hals::II2cHAL.
 */
class Ina226Driver : public IIna226Driver
{
public:
    /**
     * @brief Constructor for Ina226Driver.
     * @param i2c_hal Reference to I2C HAL implementation for DI.
     * @param config Driver configuration parameters.
     */
    Ina226Driver(idf_hals::II2cHAL& i2c_hal, const Ina226Config& config = Ina226Config());

    virtual ~Ina226Driver();

    /**
     * @brief Initializes the driver and registers the device on the provided I2C bus.
     * @param bus_handle Initialized I2C master bus handle.
     * @return ESP_OK on success.
     */
    esp_err_t init(i2c_master_bus_handle_t bus_handle) override;

    /** @copydoc IIna226Driver::init() */
    esp_err_t init() override;

    /** @copydoc IIna226Driver::reset() */
    esp_err_t reset() override;

    /** @copydoc IIna226Driver::read_shunt_voltage_uv() */
    esp_err_t read_shunt_voltage_uv(int32_t& out_uv) override;

    /** @copydoc IIna226Driver::read_bus_voltage_mv(uint16_t&) */
    esp_err_t read_bus_voltage_mv(uint16_t& out_mv) override;

    /** @copydoc IIna226Driver::read_current_ma(float&) */
    esp_err_t read_current_ma(float& out_ma) override;

    /** @copydoc IIna226Driver::read_power_mw(float&) */
    esp_err_t read_power_mw(float& out_mw) override;

    /** @copydoc IIna226Driver::calibrate() */
    esp_err_t calibrate(float r_shunt_ohms, float max_expected_current_a) override;

    /** @copydoc IIna226Driver::configure_alert() */
    esp_err_t configure_alert(uint16_t alert_mask, uint16_t alert_limit) override;

    /** @copydoc IIna226Driver::is_conversion_ready() */
    esp_err_t is_conversion_ready(bool& out_ready) override;

    /** @copydoc IIna226Driver::get_config() */
    const Ina226Config& get_config() const override { return config_; }

    /**
     * @brief Low-level register read helper for testing and direct access.
     */
    esp_err_t read_register(Register reg, uint16_t& out_val);

    /**
     * @brief Low-level register write helper for testing and direct access.
     */
    esp_err_t write_register(Register reg, uint16_t val);

private:
    idf_hals::II2cHAL& i2c_hal_;
    Ina226Config config_;
    i2c_master_dev_handle_t dev_handle_{nullptr};
    i2c_master_bus_handle_t bus_handle_{nullptr};

    float current_lsb_a_{0.000025f};
    float power_lsb_w_{0.000625f};
    uint16_t cal_val_{2048};

    esp_err_t apply_config();
};

} // namespace ina226
