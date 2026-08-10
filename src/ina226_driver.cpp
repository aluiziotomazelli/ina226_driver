#include "ina226_driver.hpp"
#include <cmath>

namespace ina226 {

Ina226Driver::Ina226Driver(idf_hals::II2cHAL& i2c_hal, const Ina226Config& config)
    : i2c_hal_(i2c_hal)
    , config_(config)
{
}

Ina226Driver::~Ina226Driver()
{
    if (dev_handle_ != nullptr) {
        i2c_hal_.master_bus_rm_device(dev_handle_);
        dev_handle_ = nullptr;
    }
}

esp_err_t Ina226Driver::init(i2c_master_bus_handle_t bus_handle)
{
    bus_handle_ = bus_handle;
    if (dev_handle_ == nullptr) {
        i2c_device_config_t dev_cfg = {};
        dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
        dev_cfg.device_address = config_.i2c_addr;
        dev_cfg.scl_speed_hz = 400000;

        esp_err_t err = i2c_hal_.master_bus_add_device(bus_handle_, &dev_cfg, &dev_handle_);
        if (err != ESP_OK) {
            return err;
        }
    }
    return init();
}

esp_err_t Ina226Driver::init()
{
    if (dev_handle_ == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    uint16_t mfg_id = 0;
    esp_err_t err = read_register(Register::MANUFACTURER_ID, mfg_id);
    if (err != ESP_OK) {
        return err;
    }
    if (mfg_id != MANUFACTURER_ID_VALUE) {
        return ESP_ERR_NOT_FOUND;
    }

    uint16_t die_id = 0;
    err = read_register(Register::DIE_ID, die_id);
    if (err != ESP_OK) {
        return err;
    }
    if (die_id != DIE_ID_VALUE) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    return apply_config();
}

esp_err_t Ina226Driver::reset()
{
    uint16_t reset_cmd = (1 << 15);
    esp_err_t err = write_register(Register::CONFIG, reset_cmd);
    if (err != ESP_OK) {
        return err;
    }
    return apply_config();
}

esp_err_t Ina226Driver::set_config(const Ina226Config& new_config)
{
    config_ = new_config;
    return apply_config();
}

esp_err_t Ina226Driver::calibrate(float r_shunt_ohms, float max_expected_current_a)
{
    if (r_shunt_ohms <= 0.0f || max_expected_current_a <= 0.0f) {
        return ESP_ERR_INVALID_ARG;
    }

    config_.r_shunt_ohms = r_shunt_ohms;
    config_.max_expected_current_a = max_expected_current_a;

    current_lsb_a_ = max_expected_current_a / 32768.0f;
    power_lsb_w_ = 25.0f * current_lsb_a_;

    double cal = 0.00512 / (static_cast<double>(current_lsb_a_) * static_cast<double>(r_shunt_ohms));
    cal_val_ = static_cast<uint16_t>(std::trunc(cal));

    return write_register(Register::CALIBRATION, cal_val_);
}

esp_err_t Ina226Driver::read_shunt_voltage_uv(int32_t& out_uv)
{
    uint16_t raw_val = 0;
    esp_err_t err = read_register(Register::SHUNT_VOLTAGE, raw_val);
    if (err != ESP_OK) {
        return err;
    }

    int16_t signed_raw = static_cast<int16_t>(raw_val);
    out_uv = static_cast<int32_t>(signed_raw * 2.5f);
    return ESP_OK;
}

esp_err_t Ina226Driver::read_bus_voltage_mv(uint16_t& out_mv)
{
    uint16_t raw_val = 0;
    esp_err_t err = read_register(Register::BUS_VOLTAGE, raw_val);
    if (err != ESP_OK) {
        return err;
    }

    out_mv = static_cast<uint16_t>(raw_val * 1.25f);
    return ESP_OK;
}

esp_err_t Ina226Driver::read_current_ma(float& out_ma)
{
    int32_t shunt_uv = 0;
    esp_err_t err = read_shunt_voltage_uv(shunt_uv);
    if (err != ESP_OK) {
        return err;
    }

    out_ma = (static_cast<float>(shunt_uv) / 1000.0f) / config_.r_shunt_ohms;
    return ESP_OK;
}

esp_err_t Ina226Driver::read_power_mw(float& out_mw)
{
    uint16_t raw_power = 0;
    esp_err_t err = read_register(Register::POWER, raw_power);
    if (err != ESP_OK) {
        return err;
    }

    out_mw = static_cast<float>(raw_power) * power_lsb_w_ * 1000.0f;
    return ESP_OK;
}

esp_err_t Ina226Driver::configure_alert(uint16_t alert_mask, uint16_t alert_limit)
{
    esp_err_t err = write_register(Register::ALERT_LIMIT, alert_limit);
    if (err != ESP_OK) {
        return err;
    }
    return write_register(Register::MASK_ENABLE, alert_mask);
}

esp_err_t Ina226Driver::is_conversion_ready(bool& out_ready)
{
    uint16_t mask_val = 0;
    esp_err_t err = read_register(Register::MASK_ENABLE, mask_val);
    if (err != ESP_OK) {
        return err;
    }

    out_ready = (mask_val & static_cast<uint16_t>(AlertFlag::CONVERSION_READY)) != 0;
    return ESP_OK;
}

esp_err_t Ina226Driver::read_register(Register reg, uint16_t& out_val)
{
    if (dev_handle_ == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t reg_addr = static_cast<uint8_t>(reg);
    uint8_t read_buf[2] = {0};

    esp_err_t err = i2c_hal_.master_transmit_receive(dev_handle_, &reg_addr, 1, read_buf, 2, 100);
    if (err != ESP_OK) {
        return err;
    }

    out_val = (static_cast<uint16_t>(read_buf[0]) << 8) | read_buf[1];
    return ESP_OK;
}

esp_err_t Ina226Driver::write_register(Register reg, uint16_t val)
{
    if (dev_handle_ == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t write_buf[3] = {
        static_cast<uint8_t>(reg), static_cast<uint8_t>((val >> 8) & 0xFF), static_cast<uint8_t>(val & 0xFF)};

    return i2c_hal_.master_transmit(dev_handle_, write_buf, 3, 100);
}

// ==================================================================
// Private
// ==================================================================
esp_err_t Ina226Driver::apply_config()
{
    uint16_t config_val = 0;
    config_val |= (static_cast<uint16_t>(config_.avg_mode) & 0x07) << 9;
    config_val |= (static_cast<uint16_t>(config_.vbus_ct) & 0x07) << 6;
    config_val |= (static_cast<uint16_t>(config_.vsh_ct) & 0x07) << 3;
    config_val |= (static_cast<uint16_t>(config_.mode) & 0x07);

    esp_err_t err = write_register(Register::CONFIG, config_val);
    if (err != ESP_OK) {
        return err;
    }

    return calibrate(config_.r_shunt_ohms, config_.max_expected_current_a);
}

} // namespace ina226
