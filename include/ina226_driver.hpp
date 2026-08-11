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
     * @brief Creates the driver bound to an I2C HAL implementation and a configuration.
     *
     * The constructor does not touch the hardware; call init() (or
     * init(bus_handle)) before performing any read/write.
     * @param[in] i2c_hal I2C HAL implementation (dependency injection, enables mocking in tests).
     * @param[in] config Driver configuration (address, shunt, calibration, conversion settings).
     */
    Ina226Driver(idf_hals::II2cHAL& i2c_hal, const Ina226Config& config = Ina226Config());

    virtual ~Ina226Driver();

    /** @copydoc IIna226Driver::init(i2c_master_bus_handle_t) */
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

    /** @copydoc IIna226Driver::set_config() */
    esp_err_t set_config(const Ina226Config& config) override;

    /** @copydoc IIna226Driver::calibrate() */
    esp_err_t calibrate(float r_shunt_ohms, float max_expected_current_a) override;

    /** @copydoc IIna226Driver::configure_alert() */
    esp_err_t configure_alert(uint16_t alert_mask, uint16_t alert_limit) override;

    /** @copydoc IIna226Driver::is_conversion_ready() */
    esp_err_t is_conversion_ready(bool& out_ready) override;

    /** @copydoc IIna226Driver::get_config() */
    const Ina226Config& get_config() const override { return config_; }

    /**
     * @brief Low-level I2C read of a 16-bit register.
     *
     * Not part of the IIna226Driver interface; exposed for testing and direct access.
     * @param[in] reg Register address to read (see Register enum).
     * @param[out] out_val 16-bit register value.
     * @return ESP_OK on success, or ESP_ERR_* on I2C failure.
     */
    esp_err_t read_register(Register reg, uint16_t& out_val);

    /**
     * @brief Low-level I2C write of a 16-bit register.
     *
     * Not part of the IIna226Driver interface; exposed for testing and direct access.
     * @param[in] reg Register address to write (see Register enum).
     * @param[in] val 16-bit value to write.
     * @return ESP_OK on success, or ESP_ERR_* on I2C failure.
     */
    esp_err_t write_register(Register reg, uint16_t val);

private:
    idf_hals::II2cHAL& i2c_hal_;                  ///< I2C HAL used for all transfers.
    Ina226Config config_;                         ///< Active configuration.
    i2c_master_dev_handle_t dev_handle_{nullptr}; ///< Registered device handle (nullptr until init()).

    float current_lsb_a_{0.000025f}; ///< Current LSB in Amperes (from calibration).
    float power_lsb_w_{0.000625f};   ///< Power LSB in Watts (= 25 * current_lsb_a_).
    uint16_t cal_val_{2048};         ///< CALIBRATION register value.

    esp_err_t apply_config(); ///< Writes CONFIG + CALIBRATION registers from config_.
};

} // namespace ina226
