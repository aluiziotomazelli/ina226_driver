#include "esp_log.h"
#include "hal_i2c.hpp"
#include "ina226_driver.hpp"

extern "C" void app_main(void)
{
    static idf_hals::I2cHAL i2c_hal;

    ina226::Ina226Config config = {
        .i2c_addr = ina226::DEFAULT_I2C_ADDR,
        .r_shunt_ohms = 0.1f,
        .max_expected_current_a = 0.8192f,
        .avg_mode = ina226::AveragingMode::AVG_16,
        .vbus_ct = ina226::ConversionTime::CT_1100US,
        .vsh_ct = ina226::ConversionTime::CT_1100US,
        .mode = ina226::OperatingMode::SHUNT_AND_BUS_CONTINUOUS
    };

    ina226::Ina226Driver ina226_driver(i2c_hal, config);
    (void)ina226_driver;
}
