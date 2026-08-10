#include <cstdio>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_sleep.h"

#include "hal_i2c.hpp"
#include "ina226_driver.hpp"

static const char* TAG = "INA226_EXAMPLE";

static constexpr gpio_num_t I2C_SDA_GPIO = GPIO_NUM_6;   // D4 on XIAO ESP32C3
static constexpr gpio_num_t I2C_SCL_GPIO = GPIO_NUM_7;   // D5 on XIAO ESP32C3
static constexpr gpio_num_t INA_ALERT_GPIO = GPIO_NUM_4; // D2 on XIAO ESP32C3

/**
 * @brief Shunt Over Voltage raw threshold
 *
 * Calculation:
 *   - Target wakeup current: I_wakeup = 1.0 mA (1000 uA)
 *   - Shunt resistor: R_shunt = 0.1 Ohm (100 mOhm)
 *   - Shunt Voltage: V_sh = 1000 uA * 0.1 Ohm = 100 uV
 *   - INA226 Shunt Voltage LSB: 2.5 uV per LSB
 *   - Raw ALERT_LIMIT = 100 uV / 2.5 uV = 40 LSBs
 */
static constexpr float WAKEUP_CURRENT_MA = 0.1f;
static constexpr float R_SHUNT_OHMS = 0.1f;
static constexpr uint16_t WAKEUP_RAW_ALERT_LIMIT =
    static_cast<uint16_t>((WAKEUP_CURRENT_MA * 1000.0f * R_SHUNT_OHMS) / 2.5f);

idf_hals::I2cHAL i2c_hal;

ina226::Ina226Config config = {
    .i2c_addr = ina226::DEFAULT_I2C_ADDR, // 0x40
    .r_shunt_ohms = R_SHUNT_OHMS,         // 100 mOhm shunt
    .max_expected_current_a = 0.8192f,    // 0.8192 A max expected current
    .avg_mode = ina226::AveragingMode::AVG_16,
    .vbus_ct = ina226::ConversionTime::CT_1100US,
    .vsh_ct = ina226::ConversionTime::CT_1100US,
    .mode = ina226::OperatingMode::SHUNT_AND_BUS_CONTINUOUS};

ina226::Ina226Driver driver(i2c_hal, config);

void enter_deep_sleep()
{
    esp_err_t err = esp_deep_sleep_enable_gpio_wakeup((1ULL << INA_ALERT_GPIO), ESP_GPIO_WAKEUP_GPIO_LOW);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable deep gpio wakeup: %s", esp_err_to_name(err));
    }
    // 1. Configure conversion time & mode for sleep monitor
    ina226::Ina226Config sleep_config = config;
    sleep_config.avg_mode = ina226::AveragingMode::AVG_1024;
    sleep_config.vbus_ct = ina226::ConversionTime::CT_8244US;
    sleep_config.vsh_ct = ina226::ConversionTime::CT_8244US;
    sleep_config.mode = ina226::OperatingMode::SHUNT_AND_BUS_CONTINUOUS;

    driver.set_config(sleep_config);

    // 2. Enable Shunt Over Voltage alert without Latch Enable (transparent mode)
    uint16_t alert_mask = static_cast<uint16_t>(ina226::AlertFlag::SHUNT_OVER_VOLTAGE);
    driver.configure_alert(alert_mask, WAKEUP_RAW_ALERT_LIMIT);

    ESP_LOGI(
        TAG, "Entering deep sleep (Wakeup limit: %u LSBs / %.2f mA)...", WAKEUP_RAW_ALERT_LIMIT, WAKEUP_CURRENT_MA);

    if (err == ESP_OK) {
        esp_deep_sleep_start();
    }
}

extern "C" void app_main(void)
{
    // Check wakeup reason
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    if (wakeup_reason == ESP_SLEEP_WAKEUP_GPIO) {
        ESP_LOGI(TAG, "Woken up from Deep Sleep by INA226 ALERT GPIO!");
    }
    else {
        ESP_LOGI(TAG, "Normal Power-On / Reset");
    }

    // Initialize I2C Master Bus
    i2c_master_bus_config_t i2c_bus_config = {};
    i2c_bus_config.i2c_port = I2C_NUM_0;
    i2c_bus_config.sda_io_num = I2C_SDA_GPIO;
    i2c_bus_config.scl_io_num = I2C_SCL_GPIO;
    i2c_bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
    i2c_bus_config.glitch_ignore_cnt = 7;
    i2c_bus_config.flags.enable_internal_pullup = true;

    i2c_master_bus_handle_t bus_handle = nullptr;
    esp_err_t err = i2c_hal.new_master_bus(&i2c_bus_config, &bus_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize I2C master bus: %s", esp_err_to_name(err));
        return;
    }

    err = driver.init(bus_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize INA226 driver: %s", esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG, "INA226 driver initialized successfully!");

    // Clear any previous ALERT latch from INA226 by reading MASK_ENABLE register upon wakeup
    uint16_t mask_enable_val = 0;
    driver.read_register(ina226::Register::MASK_ENABLE, mask_enable_val);

    // Main polling loop
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

            float current = 0.01f;
            if (current_ma < current) {
                ESP_LOGI(
                    TAG,
                    "Current (%.2f mA) is less than threshold (%.2f mA), entering deep sleep...",
                    current_ma,
                    current);
                enter_deep_sleep();
            }
        }
        else {
            ESP_LOGE(TAG, "Error reading INA226 sensors!");
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
