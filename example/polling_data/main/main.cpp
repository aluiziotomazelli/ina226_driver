#include <cstdio>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_log.h"

#include "hal_i2c.hpp"
#include "ina226_driver.hpp"

static const char* TAG = "INA226_EXAMPLE";

static constexpr gpio_num_t I2C_SDA_GPIO = GPIO_NUM_6; // D4 on XIAO ESP32C3
static constexpr gpio_num_t I2C_SCL_GPIO = GPIO_NUM_7; // D5 on XIAO ESP32C3

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Starting INA226 Polling Example...");

    // 1. Configure and initialize I2C Master Bus
    i2c_master_bus_config_t i2c_bus_config = {};
    i2c_bus_config.i2c_port = I2C_NUM_0;
    i2c_bus_config.sda_io_num = I2C_SDA_GPIO;
    i2c_bus_config.scl_io_num = I2C_SCL_GPIO;
    i2c_bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
    i2c_bus_config.glitch_ignore_cnt = 7;
    i2c_bus_config.flags.enable_internal_pullup = true;

    idf_hals::I2cHAL i2c_hal;
    i2c_master_bus_handle_t bus_handle = nullptr;

    esp_err_t err = i2c_hal.new_master_bus(&i2c_bus_config, &bus_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize I2C master bus: %s", esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG, "I2C master bus initialized (SDA: GPIO%d, SCL: GPIO%d)", I2C_SDA_GPIO, I2C_SCL_GPIO);

    // 2. Explicitly configure INA226 settings
    ina226::Ina226Config config;
    config.i2c_addr = ina226::DEFAULT_I2C_ADDR; // 0x40
    config.r_shunt_ohms = 0.1f;                 // 100 mOhm shunt
    config.max_expected_current_a = 0.8192f;    // 0.8192 A max expected current
    config.avg_mode = ina226::AveragingMode::AVG_16;
    config.vbus_ct = ina226::ConversionTime::CT_1100US;
    config.vsh_ct = ina226::ConversionTime::CT_1100US;
    config.mode = ina226::OperatingMode::SHUNT_AND_BUS_CONTINUOUS;

    // 3. Instantiate driver and initialize sensor
    ina226::Ina226Driver driver(i2c_hal, config);

    err = driver.init(bus_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize INA226 driver: %s", esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG, "INA226 driver initialized successfully!");

    // 4. Main polling loop every 1000ms
    while (true) {
        uint16_t vbus_mv = 0;
        int32_t vshunt_uv = 0;
        float current_ma = 0.0f;
        float power_mw = 0.0f;

        esp_err_t err_vbus = driver.read_bus_voltage_mv(vbus_mv);
        esp_err_t err_vsh = driver.read_shunt_voltage_uv(vshunt_uv);
        esp_err_t err_curr = driver.read_current_ma(current_ma);
        esp_err_t err_pwr = driver.read_power_mw(power_mw);

        if (err_vbus == ESP_OK && err_vsh == ESP_OK && err_curr == ESP_OK && err_pwr == ESP_OK) {
            ESP_LOGI(TAG, "--- INA226 Report ---");
            ESP_LOGI(TAG, "Bus Voltage   : %u mV (%.3f V)", vbus_mv, static_cast<float>(vbus_mv) / 1000.0f);
            ESP_LOGI(TAG, "Shunt Voltage : %ld uV (%.3f mV)", vshunt_uv, static_cast<float>(vshunt_uv) / 1000.0f);
            ESP_LOGI(TAG, "Current       : %.2f mA", current_ma);
            ESP_LOGI(TAG, "Power         : %.2f mW", power_mw);
        }
        else {
            ESP_LOGE(TAG, "Error reading INA226 sensors!");
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
