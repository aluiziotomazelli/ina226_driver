#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "ina226_driver.hpp"
#include "mock_hal_i2c.hpp"

using ::testing::_;
using ::testing::DoAll;
using ::testing::Invoke;
using ::testing::NiceMock;
using ::testing::Return;
using ::testing::SetArrayArgument;

class Ina226DriverTest : public ::testing::Test
{
protected:
    NiceMock<idf_hals::MockI2cHAL> mock_i2c;
    ina226::Ina226Config config;
    std::unique_ptr<ina226::Ina226Driver> driver;

    i2c_master_bus_handle_t dummy_bus = reinterpret_cast<i2c_master_bus_handle_t>(0x1234);
    i2c_master_dev_handle_t dummy_dev = reinterpret_cast<i2c_master_dev_handle_t>(0x5678);

    void SetUp() override
    {
        config.i2c_addr = 0x40;
        config.r_shunt_ohms = 0.1f;
        config.max_expected_current_a = 0.8192f;

        ON_CALL(mock_i2c, master_bus_add_device(_, _, _))
            .WillByDefault(Invoke(
                [this](i2c_master_bus_handle_t, const i2c_device_config_t*, i2c_master_dev_handle_t* ret_handle) {
                    *ret_handle = dummy_dev;
                    return ESP_OK;
                }));

        driver = std::make_unique<ina226::Ina226Driver>(mock_i2c, config);
    }

    void helper_mock_register_read(uint8_t reg, uint16_t val)
    {
        uint8_t msb = (val >> 8) & 0xFF;
        uint8_t lsb = val & 0xFF;

        EXPECT_CALL(mock_i2c, master_transmit_receive(dummy_dev, _, 1, _, 2, _))
            .WillOnce(
                Invoke([reg, msb, lsb](
                           i2c_master_dev_handle_t, const uint8_t* write_buf, size_t, uint8_t* read_buf, size_t, int) {
                    if (write_buf[0] == reg) {
                        read_buf[0] = msb;
                        read_buf[1] = lsb;
                        return ESP_OK;
                    }
                    return ESP_FAIL;
                }));
    }
};

TEST_F(Ina226DriverTest, InitSuccessVerifiedManufacturerAndDieId)
{
    // Mock Manufacturer ID (0xFE -> 0x5449 "TI")
    // Mock Die ID (0xFF -> 0x2260)
    EXPECT_CALL(mock_i2c, master_transmit_receive(dummy_dev, _, 1, _, 2, _))
        .WillOnce(Invoke([](i2c_master_dev_handle_t, const uint8_t* write_buf, size_t, uint8_t* read_buf, size_t, int) {
            EXPECT_EQ(write_buf[0], static_cast<uint8_t>(ina226::Register::MANUFACTURER_ID));
            read_buf[0] = 0x54;
            read_buf[1] = 0x49;
            return ESP_OK;
        }))
        .WillOnce(Invoke([](i2c_master_dev_handle_t, const uint8_t* write_buf, size_t, uint8_t* read_buf, size_t, int) {
            EXPECT_EQ(write_buf[0], static_cast<uint8_t>(ina226::Register::DIE_ID));
            read_buf[0] = 0x22;
            read_buf[1] = 0x60;
            return ESP_OK;
        }));

    // Expect configuration write to CONFIG and CALIBRATION registers
    EXPECT_CALL(mock_i2c, master_transmit(dummy_dev, _, 3, _))
        .Times(2) // CONFIG and CALIBRATION
        .WillRepeatedly(Return(ESP_OK));

    EXPECT_EQ(driver->init(dummy_bus), ESP_OK);
}

TEST_F(Ina226DriverTest, InitFailsOnWrongManufacturerId)
{
    EXPECT_CALL(mock_i2c, master_transmit_receive(dummy_dev, _, 1, _, 2, _))
        .WillOnce(Invoke([](i2c_master_dev_handle_t, const uint8_t*, size_t, uint8_t* read_buf, size_t, int) {
            read_buf[0] = 0xFF;
            read_buf[1] = 0xFF; // Wrong ID
            return ESP_OK;
        }));

    EXPECT_EQ(driver->init(dummy_bus), ESP_ERR_NOT_FOUND);
}

TEST_F(Ina226DriverTest, InitFailsIfMasterBusAddDeviceFails)
{
    EXPECT_CALL(mock_i2c, master_bus_add_device(_, _, _)).WillOnce(Return(ESP_FAIL));
    EXPECT_EQ(driver->init(dummy_bus), ESP_FAIL);
}

TEST_F(Ina226DriverTest, ReadShuntVoltagePositiveAndNegative)
{
    driver->init(dummy_bus);

    // Test positive shunt voltage: 3000 * 2.5uV = 7500 uV (7.5 mV)
    helper_mock_register_read(static_cast<uint8_t>(ina226::Register::SHUNT_VOLTAGE), 3000);
    int32_t uv = 0;
    EXPECT_EQ(driver->read_shunt_voltage_uv(uv), ESP_OK);
    EXPECT_EQ(uv, 7500);

    // Test negative shunt voltage: -3000 (0xF448) * 2.5uV = -7500 uV (-7.5 mV)
    helper_mock_register_read(static_cast<uint8_t>(ina226::Register::SHUNT_VOLTAGE), 0xF448);
    EXPECT_EQ(driver->read_shunt_voltage_uv(uv), ESP_OK);
    EXPECT_EQ(uv, -7500);
}

TEST_F(Ina226DriverTest, ReadCurrentMaFromShuntResistor)
{
    driver->init(dummy_bus);

    // 70000 uV (70 mV) across 0.1 ohm shunt = 700 mA (0.7 A)
    // 70000 uV / 2.5 uV/LSB = 28000 LSB = 0x6D60
    helper_mock_register_read(static_cast<uint8_t>(ina226::Register::SHUNT_VOLTAGE), 28000);

    float current_ma = 0.0f;
    EXPECT_EQ(driver->read_current_ma(current_ma), ESP_OK);
    EXPECT_NEAR(current_ma, 700.0f, 0.01f);
}

TEST_F(Ina226DriverTest, ReadBusVoltageMv)
{
    driver->init(dummy_bus);

    // 7200 * 1.25 mV = 9000 mV (9.0V)
    helper_mock_register_read(static_cast<uint8_t>(ina226::Register::BUS_VOLTAGE), 7200);

    uint16_t bus_mv = 0;
    EXPECT_EQ(driver->read_bus_voltage_mv(bus_mv), ESP_OK);
    EXPECT_EQ(bus_mv, 9000);
}

TEST_F(Ina226DriverTest, ConfigureAlertAndCheckConversionReady)
{
    driver->init(dummy_bus);

    EXPECT_CALL(mock_i2c, master_transmit(dummy_dev, _, 3, _))
        .WillOnce(Invoke([](i2c_master_dev_handle_t, const uint8_t* write_buf, size_t, int) {
            EXPECT_EQ(write_buf[0], static_cast<uint8_t>(ina226::Register::ALERT_LIMIT));
            return ESP_OK;
        }))
        .WillOnce(Invoke([](i2c_master_dev_handle_t, const uint8_t* write_buf, size_t, int) {
            EXPECT_EQ(write_buf[0], static_cast<uint8_t>(ina226::Register::MASK_ENABLE));
            uint16_t mask = (write_buf[1] << 8) | write_buf[2];
            EXPECT_EQ(mask, static_cast<uint16_t>(ina226::AlertFlag::ALERT_ON_CONVERSION_READY));
            return ESP_OK;
        }));

    EXPECT_EQ(driver->configure_alert(
                  static_cast<uint16_t>(ina226::AlertFlag::ALERT_ON_CONVERSION_READY), 0),
              ESP_OK);

    // Regression: CNVR enable bit (bit 10) alone must NOT report conversion ready
    helper_mock_register_read(static_cast<uint8_t>(ina226::Register::MASK_ENABLE),
                              static_cast<uint16_t>(ina226::AlertFlag::ALERT_ON_CONVERSION_READY));
    bool ready = true;
    EXPECT_EQ(driver->is_conversion_ready(ready), ESP_OK);
    EXPECT_FALSE(ready);

    // CVRF flag (bit 3) set -> conversion ready
    helper_mock_register_read(static_cast<uint8_t>(ina226::Register::MASK_ENABLE),
                              static_cast<uint16_t>(ina226::AlertFlag::CONVERSION_READY));
    EXPECT_EQ(driver->is_conversion_ready(ready), ESP_OK);
    EXPECT_TRUE(ready);
}

TEST_F(Ina226DriverTest, SetConfigSavesConfigurationAndApplysIt)
{
    driver->init(dummy_bus);

    ina226::Ina226Config config = {};
    config = driver->get_config();

    ina226::Ina226Config new_config = config;
    new_config.r_shunt_ohms = 0.2f;
    new_config.max_expected_current_a = 0.4023f;
    new_config.avg_mode = ina226::AveragingMode::AVG_128;
    new_config.vbus_ct = ina226::ConversionTime::CT_8244US;
    new_config.vsh_ct = ina226::ConversionTime::CT_8244US;

    new_config.mode = ina226::OperatingMode::SHUNT_AND_BUS_TRIGGERED;

    // Expect configuration write to CONFIG and CALIBRATION registers
    EXPECT_CALL(mock_i2c, master_transmit(dummy_dev, _, 3, _))
        .Times(2) // CONFIG and CALIBRATION
        .WillRepeatedly(Return(ESP_OK));

    EXPECT_EQ(driver->set_config(new_config), ESP_OK);

    ina226::Ina226Config read_config = driver->get_config();
    EXPECT_EQ(read_config.r_shunt_ohms, new_config.r_shunt_ohms);
    EXPECT_EQ(read_config.max_expected_current_a, new_config.max_expected_current_a);
    EXPECT_EQ(read_config.avg_mode, new_config.avg_mode);
    EXPECT_EQ(read_config.vbus_ct, new_config.vbus_ct);
    EXPECT_EQ(read_config.vsh_ct, new_config.vsh_ct);
    EXPECT_EQ(read_config.mode, new_config.mode);

    EXPECT_NE(read_config.r_shunt_ohms, config.r_shunt_ohms);
}

TEST_F(Ina226DriverTest, ResetSuccessAppliesResetBitAndReappliesConfig)
{
    driver->init(dummy_bus);

    // 1st transmit: reset command (0x8000 / bit 15 set) written to CONFIG register
    // 2nd transmit: apply_config writes new CONFIG register based on config_
    // 3rd transmit: apply_config writes CALIBRATION register
    ::testing::InSequence seq;

    EXPECT_CALL(mock_i2c, master_transmit(dummy_dev, _, 3, _))
        .WillOnce(Invoke([](i2c_master_dev_handle_t, const uint8_t* write_buf, size_t write_size, int) {
            EXPECT_EQ(write_size, 3);
            EXPECT_EQ(write_buf[0], static_cast<uint8_t>(ina226::Register::CONFIG));
            uint16_t sent_val = (static_cast<uint16_t>(write_buf[1]) << 8) | write_buf[2];
            EXPECT_EQ(sent_val, 1 << 15); // Bit 15 set for Reset
            return ESP_OK;
        }))
        .WillOnce(Invoke([](i2c_master_dev_handle_t, const uint8_t* write_buf, size_t write_size, int) {
            EXPECT_EQ(write_size, 3);
            EXPECT_EQ(write_buf[0], static_cast<uint8_t>(ina226::Register::CONFIG));
            return ESP_OK;
        }))
        .WillOnce(Invoke([](i2c_master_dev_handle_t, const uint8_t* write_buf, size_t write_size, int) {
            EXPECT_EQ(write_size, 3);
            EXPECT_EQ(write_buf[0], static_cast<uint8_t>(ina226::Register::CALIBRATION));
            return ESP_OK;
        }));

    EXPECT_EQ(driver->reset(), ESP_OK);
}

TEST_F(Ina226DriverTest, ReadPowerMwSuccess)
{
    driver->init(dummy_bus);

    // Default configuration set in SetUp():
    // max_expected_current_a = 0.8192 A
    // current_lsb_a_ = 0.8192 / 32768 = 25 uA (0.000025 A)
    // power_lsb_w_ = 25 * current_lsb_a_ = 0.000625 W (0.625 mW per LSB)
    // For raw_power = 1000:
    // power_mw = 1000 * 0.000625 W * 1000 mW/W = 625.0 mW
    helper_mock_register_read(static_cast<uint8_t>(ina226::Register::POWER), 1000);

    float power_mw = 0.0f;
    EXPECT_EQ(driver->read_power_mw(power_mw), ESP_OK);
    EXPECT_NEAR(power_mw, 625.0f, 0.01f);
}
