#include <cstdio>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_system.h"

#include "hal_i2c.hpp"
#include "ina226_driver.hpp"

static const char* TAG = "INA226_EXAMPLE";

static constexpr gpio_num_t I2C_SDA_GPIO = GPIO_NUM_6;   // D4 on XIAO ESP32C3
static constexpr gpio_num_t I2C_SCL_GPIO = GPIO_NUM_7;   // D5 on XIAO ESP32C3
static constexpr gpio_num_t INA_ALERT_GPIO = GPIO_NUM_3; // D2 on XIAO ESP32C3
static constexpr gpio_num_t LED_GPIO = GPIO_NUM_20;      // Visual indicator (active-high LED)

/// Width of a short LED blink (reading indicator).
static constexpr uint32_t LED_BLINK_MS = 50;
/// Gap between blinks in a multi-blink pattern.
static constexpr uint32_t LED_GAP_MS = 100;
/// Width of the single pulse shown right before entering deep sleep.
static constexpr uint32_t LED_SLEEP_PULSE_MS = 400;

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

/// Sleep only when the measured current is below this threshold.
static constexpr float SLEEP_CURRENT_THRESHOLD_MA = 0.01f;

/// Safety-net timer wakeup: if the GPIO (ALERT) wakeup is ever missed (known
/// ESP32-C3 RTC-GPIO quirks, e.g. espressif/esp-idf#8449), the chip still
/// wakes periodically to re-check the current and re-arm the alert. It also
/// proves the chip really re-entered deep sleep (see the boot log).
static constexpr uint64_t WAKEUP_TIMER_US = 1ULL * 60ULL * 1000000ULL;

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

const char* reset_reason_str()
{
    switch (esp_reset_reason()) {
    case ESP_RST_POWERON:
        return "POWER-ON";
    case ESP_RST_EXT:
        return "EXTERNAL PIN";
    case ESP_RST_SW:
        return "SOFTWARE";
    case ESP_RST_PANIC:
        return "PANIC";
    case ESP_RST_INT_WDT:
        return "INT WATCHDOG";
    case ESP_RST_TASK_WDT:
        return "TASK WATCHDOG";
    case ESP_RST_WDT:
        return "WATCHDOG";
    case ESP_RST_DEEPSLEEP:
        return "DEEP SLEEP WAKEUP";
    case ESP_RST_BROWNOUT:
        return "BROWNOUT";
    case ESP_RST_PWR_GLITCH:
        return "POWER GLITCH";
    default:
        return "UNKNOWN";
    }
}

void init_led()
{
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1ULL << LED_GPIO);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);
    gpio_set_level(LED_GPIO, 0);
}

void led_pulse(uint32_t on_ms)
{
    gpio_set_level(LED_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(on_ms));
    gpio_set_level(LED_GPIO, 0);
}

void led_blinks(int count)
{
    for (int i = 0; i < count; ++i) {
        led_pulse(LED_BLINK_MS);
        if (i + 1 < count) {
            vTaskDelay(pdMS_TO_TICKS(LED_GAP_MS));
        }
    }
}

void enter_deep_sleep()
{
    // 1. Configure conversion time & mode for sleep monitor
    ina226::Ina226Config sleep_config = config;
    sleep_config.avg_mode = ina226::AveragingMode::AVG_1024;
    sleep_config.vbus_ct = ina226::ConversionTime::CT_8244US;
    sleep_config.vsh_ct = ina226::ConversionTime::CT_8244US;
    sleep_config.mode = ina226::OperatingMode::SHUNT_AND_BUS_CONTINUOUS;
    driver.set_config(sleep_config);

    // 2. Enable Shunt Over Voltage alert in TRANSPARENT mode (LEN=0). This is
    //    a solar sensor: the long averaging window (AVG_1024 + 8244us -> ~16.9s
    //    per full cycle) already filters out short transients, and without the
    //    latch the ALERT pin tracks the condition in real time, so a brief
    //    glitch (e.g. a lightning spike) cannot hold the pin low and force
    //    repeated wakeups.
    uint16_t alert_mask = static_cast<uint16_t>(ina226::AlertFlag::SHUNT_OVER_VOLTAGE);
    esp_err_t err = driver.configure_alert(alert_mask, WAKEUP_RAW_ALERT_LIMIT);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure INA226 alert: %s", esp_err_to_name(err));
        return;
    }

    // 3. Clear any stale alert flag. The MASK_ENABLE flags (AFF/CVRF) are
    //    read-to-clear; in transparent mode the pin itself is not latched, it
    //    already tracks the current condition.
    uint16_t alert_flags = 0;
    driver.read_alert_flags(alert_flags);

    // 4. Sanity check: the pin must be HIGH (alert de-asserted) right before
    //    sleeping. With a level-triggered LOW wakeup, entering sleep while the
    //    pin is LOW would wake the chip up immediately (wake/sleep loop, which
    //    can look like "reset does not work"). We only sleep when the current
    //    is below threshold, so a LOW pin here means the INA226 is still
    //    evaluating a previous condition - skip this cycle and keep polling.
    if (gpio_get_level(INA_ALERT_GPIO) == 0) {
        ESP_LOGW(TAG, "ALERT pin is LOW at sleep entry, skipping deep sleep this cycle");
        return;
    }

    // 5. Arm wakeup sources: GPIO (ALERT, active-low) + timer (safety net).
    err = esp_deep_sleep_enable_gpio_wakeup((1ULL << INA_ALERT_GPIO), ESP_GPIO_WAKEUP_GPIO_LOW);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable deep gpio wakeup: %s", esp_err_to_name(err));
        return;
    }
    err = esp_sleep_enable_timer_wakeup(WAKEUP_TIMER_US);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable timer wakeup: %s", esp_err_to_name(err));
    }

    // Short settle time so the INA226 I2C writes are fully drained and the RTC
    // wakeup sources are armed before the power domain drops.
    vTaskDelay(pdMS_TO_TICKS(100));

    // Visual: single long pulse = "going to sleep".
    led_pulse(LED_SLEEP_PULSE_MS);

    ESP_LOGI(
        TAG,
        "Entering deep sleep (Wakeup limit: %u LSBs / %.2f mA, ALERT pin level: %d)...",
        WAKEUP_RAW_ALERT_LIMIT,
        WAKEUP_CURRENT_MA,
        gpio_get_level(INA_ALERT_GPIO));

    esp_deep_sleep_start();
}

extern "C" void app_main(void)
{
    // Boot diagnostics: what brought us here and what is the ALERT pin doing.
    // This distinguishes deep-sleep wakeup, power-on, brownout, and the
    // ESP32-C3 case where the chip wakes and re-sleeps so fast that the user
    // believes the reset button "does not work".
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    ESP_LOGI(TAG, "Reset reason: %s", reset_reason_str());
    ESP_LOGI(TAG, "ALERT pin level at boot: %d", gpio_get_level(INA_ALERT_GPIO));
    if (wakeup_reason == ESP_SLEEP_WAKEUP_GPIO) {
        ESP_LOGI(TAG, "Woken up from Deep Sleep by INA226 ALERT GPIO!");
    }
    else if (wakeup_reason == ESP_SLEEP_WAKEUP_TIMER) {
        ESP_LOGI(TAG, "Woken up from Deep Sleep by safety-net timer (GPIO wakeup was missed)");
    }
    else if (wakeup_reason == ESP_SLEEP_WAKEUP_UNDEFINED) {
        ESP_LOGI(TAG, "Normal Power-On / Reset");
    }
    else {
        ESP_LOGI(TAG, "Unexpected wakeup cause: %d", static_cast<int>(wakeup_reason));
    }

    // Visual debug: the serial monitor cannot be trusted during deep sleep
    // (the USB-Serial-JTAG is powered down and re-enumeration is slow/flaky),
    // so the LED is the primary feedback. Pattern shown at boot:
    //   1 short blink = power-on / reset
    //   2 short blinks = GPIO wakeup (ALERT: current above limit)
    //   3 short blinks = timer wakeup (safety net)
    init_led();
    if (wakeup_reason == ESP_SLEEP_WAKEUP_GPIO) {
        led_blinks(2);
    }
    else if (wakeup_reason == ESP_SLEEP_WAKEUP_TIMER) {
        led_blinks(3);
    }
    else {
        led_blinks(1);
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

    driver.set_config(config);

    // Clear any previous alert flag from INA226 upon wakeup: read_alert_flags()
    // reads MASK_ENABLE, whose AFF/CVRF flags are read-to-clear.
    uint16_t alert_flags = 0;
    driver.read_alert_flags(alert_flags);
    ESP_LOGI(
        TAG, "MASK_ENABLE: 0x%04X (ALERT pin level after clear: %d)", alert_flags, gpio_get_level(INA_ALERT_GPIO));

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

            // Visual confirmation that a reading happened.
            led_pulse(LED_BLINK_MS);

            if (current_ma < SLEEP_CURRENT_THRESHOLD_MA) {
                ESP_LOGI(
                    TAG,
                    "Current (%.2f mA) is less than threshold (%.2f mA), entering deep sleep...",
                    current_ma,
                    SLEEP_CURRENT_THRESHOLD_MA);
                enter_deep_sleep();
                // If we get here, the sleep was skipped (e.g. ALERT pin was LOW
                // at sleep entry). Keep polling instead of busy-looping.
            }
        }
        else {
            ESP_LOGE(TAG, "Error reading INA226 sensors!");
            led_blinks(5);
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
