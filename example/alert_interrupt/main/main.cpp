#include <cstdio>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_log.h"

#include "hal_i2c.hpp"
#include "ina226_driver.hpp"

static const char* TAG = "INA226_ALERT_EXAMPLE";

static constexpr gpio_num_t I2C_SDA_GPIO = GPIO_NUM_6;    // D4
static constexpr gpio_num_t I2C_SCL_GPIO = GPIO_NUM_7;    // D5
static constexpr gpio_num_t INA_ALERT_GPIO = GPIO_NUM_10; // D10

static SemaphoreHandle_t alert_semaphore = nullptr;

static void IRAM_ATTR alert_gpio_isr_handler(void* arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(alert_semaphore, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Starting INA226 Alert Interrupt Example...");

    // 1. Create binary semaphore for ISR notification
    alert_semaphore = xSemaphoreCreateBinary();
    if (alert_semaphore == nullptr) {
        ESP_LOGE(TAG, "Failed to create alert semaphore");
        return;
    }

    // 2. Configure GPIO interrupt for INA_ALERT_GPIO (Active Low -> Falling Edge)
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.pin_bit_mask = (1ULL << INA_ALERT_GPIO);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_config(&io_conf);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(INA_ALERT_GPIO, alert_gpio_isr_handler, nullptr);

    // 3. Configure and initialize I2C Master Bus
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

    // 4. Configure INA226: AVG_128, CT_4156US (~1Hz total conversion rate)
    ina226::Ina226Config config;
    config.i2c_addr = ina226::DEFAULT_I2C_ADDR; // 0x40
    config.r_shunt_ohms = 0.1f;                 // 100 mOhm shunt
    config.max_expected_current_a = 1.0f;       // 1.0 A max expected current
    config.avg_mode = ina226::AveragingMode::AVG_128;
    config.vbus_ct = ina226::ConversionTime::CT_4156US;
    config.vsh_ct = ina226::ConversionTime::CT_4156US;
    config.mode = ina226::OperatingMode::SHUNT_AND_BUS_CONTINUOUS;

    // 5. Instantiate driver and initialize sensor
    ina226::Ina226Driver driver(i2c_hal, config);

    err = driver.init(bus_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize INA226 driver: %s", esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG, "INA226 driver initialized successfully!");

    // 6. Configure INA226 Alert pin for CONVERSION_READY flag
    uint16_t alert_mask = static_cast<uint16_t>(ina226::AlertFlag::CONVERSION_READY);
    err = driver.configure_alert(alert_mask, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure INA226 ALERT pin: %s", esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG, "INA226 ALERT pin configured for CONVERSION_READY interrupt on GPIO%d", INA_ALERT_GPIO);

    // 7. Main loop waiting for ISR semaphore notifications
    while (true) {
        if (xSemaphoreTake(alert_semaphore, portMAX_DELAY) == pdTRUE) {
            uint16_t vbus_mv = 0;
            int32_t vshunt_uv = 0;
            float current_ma = 0.0f;
            float power_mw = 0.0f;

            esp_err_t err_vbus = driver.read_bus_voltage_mv(vbus_mv);
            esp_err_t err_vsh = driver.read_shunt_voltage_uv(vshunt_uv);
            esp_err_t err_curr = driver.read_current_ma(current_ma);
            esp_err_t err_pwr = driver.read_power_mw(power_mw);

            if (err_vbus == ESP_OK && err_vsh == ESP_OK && err_curr == ESP_OK && err_pwr == ESP_OK) {
                ESP_LOGI(TAG, "[ALERT ISR] Conversion Ready!");
                ESP_LOGI(TAG, "Bus Voltage   : %u mV (%.3f V)", vbus_mv, static_cast<float>(vbus_mv) / 1000.0f);
                ESP_LOGI(TAG, "Shunt Voltage : %ld uV (%.3f mV)", vshunt_uv, static_cast<float>(vshunt_uv) / 1000.0f);
                ESP_LOGI(TAG, "Current       : %.2f mA", current_ma);
                ESP_LOGI(TAG, "Power         : %.2f mW", power_mw);
            }
            else {
                ESP_LOGE(TAG, "Error reading INA226 sensors on alert interrupt!");
            }
        }
    }
}
