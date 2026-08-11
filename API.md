# INA226 Driver Component API Reference

Detailed programming interface reference for the `ina226_driver` component.

---

## Data Types

### `DEFAULT_I2C_ADDR`
```cpp
static constexpr uint8_t DEFAULT_I2C_ADDR = 0x40; ///< Default 7-bit I2C address (A0 = A1 = GND).
```
Default INA226 7-bit I2C address when both address pins are tied to GND.

### `MANUFACTURER_ID_VALUE`
```cpp
static constexpr uint16_t MANUFACTURER_ID_VALUE = 0x5449; ///< Expected MANUFACTURER_ID register value ("TI").
```
Expected value of the MANUFACTURER_ID register (0xFE), used to verify the device is a Texas Instruments INA226.

### `DIE_ID_VALUE`
```cpp
static constexpr uint16_t DIE_ID_VALUE = 0x2260; ///< Expected DIE_ID register value.
```
Expected value of the DIE_ID register (0xFF), used to verify the device is an INA226.

### `RESET_BIT`
```cpp
static constexpr uint16_t RESET_BIT = (1 << 15); ///< RST: configuration reset bit (self-clearing).
```
Bit 15 of the CONFIG register. Writing 1 triggers a soft reset; the bit reads back as 1 while the reset is in progress and self-clears to 0 once the device returns to its power-on state.

### `Register`
```cpp
enum class Register : uint8_t {
    CONFIG = 0x00,          ///< Configuration register
    SHUNT_VOLTAGE = 0x01,   ///< Shunt voltage measurement
    BUS_VOLTAGE = 0x02,     ///< Bus voltage measurement
    POWER = 0x03,           ///< Power measurement
    CURRENT = 0x04,         ///< Current measurement
    CALIBRATION = 0x05,     ///< Calibration register
    MASK_ENABLE = 0x06,     ///< Interrupt mask and flag enable
    ALERT_LIMIT = 0x07,     ///< Alert limit register
    MANUFACTURER_ID = 0xFE, ///< Manufacturer ID (0x5449)
    DIE_ID = 0xFF           ///< Die ID (0x2260)
};
```
INA226 register map (datasheet SBOS448B).

### `AveragingMode`
```cpp
enum class AveragingMode : uint16_t {
    AVG_1 = 0b000,
    AVG_4 = 0b001,
    AVG_16 = 0b010,
    AVG_64 = 0b011,
    AVG_128 = 0b100,
    AVG_256 = 0b101,
    AVG_512 = 0b110,
    AVG_1024 = 0b111
};
```
Number of samples averaged per conversion (AVG bits D11:D9 of the CONFIG register).

### `averaging_mode_to_count()`
```cpp
constexpr uint16_t averaging_mode_to_count(AveragingMode mode)
```
- **Parameters**: `mode` - Averaging mode.
- **Returns**: Number of averaged samples; returns `1` for an unknown value.

### `ConversionTime`
```cpp
enum class ConversionTime : uint16_t {
    CT_140US = 0b000,
    CT_204US = 0b001,
    CT_332US = 0b010,
    CT_588US = 0b011,
    CT_1100US = 0b100,
    CT_2116US = 0b101,
    CT_4156US = 0b110,
    CT_8244US = 0b111
};
```
Conversion time for VBUS or VSHUNT (VBUSCT/VSHCT bits D8:D6 and D5:D3 of the CONFIG register).

### `conversion_time_to_us()`
```cpp
constexpr uint32_t conversion_time_to_us(ConversionTime ct)
```
- **Parameters**: `ct` - Conversion time setting.
- **Returns**: Duration in microseconds; returns `1100` us for an unknown value.

### `OperatingMode`
```cpp
enum class OperatingMode : uint16_t {
    POWER_DOWN = 0b000,
    SHUNT_TRIGGERED = 0b001,
    BUS_TRIGGERED = 0b010,
    SHUNT_AND_BUS_TRIGGERED = 0b011,
    ADC_OFF = 0b100,
    SHUNT_CONTINUOUS = 0b101,
    BUS_CONTINUOUS = 0b110,
    SHUNT_AND_BUS_CONTINUOUS = 0b111
};
```
ADC operating mode; matches the MODE bits (D2:D0) of the CONFIG register.

### `AlertFlag`
```cpp
enum class AlertFlag : uint16_t {
    SHUNT_OVER_VOLTAGE = (1 << 15),        ///< SOL: Shunt Voltage Over-Voltage
    SHUNT_UNDER_VOLTAGE = (1 << 14),       ///< SUL: Shunt Voltage Under-Voltage
    BUS_OVER_VOLTAGE = (1 << 13),          ///< BOL: Bus Voltage Over-Voltage
    BUS_UNDER_VOLTAGE = (1 << 12),         ///< BUL: Bus Voltage Under-Voltage
    POWER_OVER_LIMIT = (1 << 11),          ///< POL: Power Over-Limit
    ALERT_ON_CONVERSION_READY = (1 << 10), ///< CNVR: assert Alert pin when a conversion completes
    ALERT_FUNCTION_FLAG = (1 << 4),        ///< AFF: Alert Function Flag (read-to-clear)
    CONVERSION_READY = (1 << 3),           ///< CVRF: conversion-ready status flag (read-to-clear)
    ALERT_POLARITY_HIGH = (1 << 1),        ///< APOL: 1 = active-high, 0 = active-low (default)
    LATCH_ENABLE = (1 << 0)                ///< LEN: latch the Alert pin until MASK_ENABLE is read
};
```
Bitmasks for the MASK_ENABLE register (datasheet SBOS448B).

### `Ina226Config`
```cpp
struct Ina226Config {
    uint8_t i2c_addr = DEFAULT_I2C_ADDR;              ///< 7-bit I2C device address.
    float r_shunt_ohms = 0.1f;                        ///< Shunt resistance in Ohms (e.g. 0.1 for R100).
    float max_expected_current_a = 0.8192f;           ///< Maximum expected current in Amperes (defines Current_LSB / CAL).
    AveragingMode avg_mode = AveragingMode::AVG_16;   ///< Samples averaged per conversion.
    ConversionTime vbus_ct = ConversionTime::CT_1100US; ///< VBUS conversion time.
    ConversionTime vsh_ct = ConversionTime::CT_1100US; ///< VSHUNT conversion time.
    OperatingMode mode = OperatingMode::SHUNT_AND_BUS_CONTINUOUS; ///< ADC operating mode.
};
```
Configuration parameters for initializing the INA226. `r_shunt_ohms` and `max_expected_current_a` define the measurement resolution: `Current_LSB = max_expected_current_a / 32768`, and the resulting CALIBRATION value (`CAL = 0.00512 / (Current_LSB * r_shunt_ohms)`) must fit in 15 bits (see `Ina226Driver::calibrate()`).

---

## Class Reference

### `IIna226Driver` (Abstract Interface)
Defined in `ina226_driver/include/interfaces/i_ina226_driver.hpp`.

#### `virtual esp_err_t init(i2c_master_bus_handle_t bus_handle) = 0`
Adds the INA226 device to an initialized I2C master bus and performs hardware initialization (chip ID verification + configuration write). This is the entry point for normal use; it internally calls `init()` after registering the device on the bus.
- **Parameters**: `bus_handle` - Initialized I2C master bus handle.
- **Returns**: `ESP_OK` on success, or `ESP_ERR_*` on I2C failure / ID mismatch.

#### `virtual esp_err_t init() = 0`
Verifies the INA226 chip IDs and writes the current configuration. Requires the device to be registered on a bus first (see `init(bus_handle)`).
- **Returns**: `ESP_OK` on success, `ESP_ERR_INVALID_STATE` if no device is registered, `ESP_ERR_NOT_FOUND` if the manufacturer ID does not match, `ESP_ERR_INVALID_RESPONSE` if the die ID does not match, or `ESP_ERR_*` on I2C failure.

#### `virtual esp_err_t reset() = 0`
Performs a soft reset of the INA226 registers. Polls the self-clearing RST bit (datasheet SBOS448B) until it reads back 0, then re-applies the active configuration and calibration.
- **Returns**: `ESP_OK` on success, `ESP_ERR_TIMEOUT` if the RST bit never clears, or `ESP_ERR_*` on I2C failure.

#### `virtual esp_err_t read_shunt_voltage_uv(int32_t& out_uv) = 0`
Reads the shunt voltage in microvolts (uV).
- **Parameters**: `out_uv` - Measured shunt voltage; signed (negative = reverse current direction).
- **Returns**: `ESP_OK` on success, or `ESP_ERR_*` on I2C failure.

#### `virtual esp_err_t read_bus_voltage_mv(uint16_t& out_mv) = 0`
Reads the bus voltage in millivolts (mV).
- **Parameters**: `out_mv` - Measured bus voltage; unsigned (INA226 bus range is 0-36 V).
- **Returns**: `ESP_OK` on success, or `ESP_ERR_*` on I2C failure.

#### `virtual esp_err_t read_current_ma(float& out_ma) = 0`
Reads the current in milliamperes (mA), as computed by the INA226. Reads the CURRENT register (04h) and scales by the Current_LSB derived at calibration time. Requires a valid calibration to have been applied (see `calibrate()` / `Ina226Config`).
- **Parameters**: `out_ma` - Measured current in mA; signed (bidirectional measurement).
- **Returns**: `ESP_OK` on success, or `ESP_ERR_*` on I2C failure.

#### `virtual esp_err_t read_power_mw(float& out_mw) = 0`
Reads the power in milliwatts (mW), as computed by the INA226.
- **Parameters**: `out_mw` - Measured power in mW.
- **Returns**: `ESP_OK` on success, or `ESP_ERR_*` on I2C failure.

#### `virtual esp_err_t set_config(const Ina226Config& new_config) = 0`
Applies a new configuration and recalibrates the device. Writes the CONFIG register from the new parameters and recomputes the CALIBRATION register from `r_shunt_ohms` / `max_expected_current_a`.
- **Parameters**: `new_config` - Configuration to apply.
- **Returns**: `ESP_OK` on success, `ESP_ERR_INVALID_ARG` if the computed CAL value exceeds the 15-bit CALIBRATION register, or `ESP_ERR_*` on I2C failure.

#### `virtual esp_err_t calibrate(float r_shunt_ohms, float max_expected_current_a) = 0`
Recomputes and writes the CALIBRATION register. `Current_LSB = max_expected_current_a / 32768` and `CAL = 0.00512 / (Current_LSB * r_shunt_ohms)`. CAL must fit in the 15 writeable bits of the CALIBRATION register (max 0x7FFF, datasheet SBOS448B).
- **Parameters**:
  - `r_shunt_ohms` - Shunt resistance in Ohms (must be > 0).
  - `max_expected_current_a` - Maximum expected current in Amperes (must be > 0).
- **Returns**: `ESP_OK` on success, `ESP_ERR_INVALID_ARG` if any argument is <= 0 or the computed CAL exceeds 0x7FFF, or `ESP_ERR_*` on I2C failure.

#### `virtual esp_err_t configure_alert(uint16_t alert_mask, uint16_t alert_limit) = 0`
Configures the alert threshold and the MASK_ENABLE bits. Writes ALERT_LIMIT (07h) with `alert_limit`, then MASK_ENABLE (06h) with the given `AlertFlag` bitmask. The ALERT pin is open-drain and active-low by default (APOL = 0); it asserts while any enabled alert condition is true (or stays asserted until MASK_ENABLE is read when LEN = 1).
- **Parameters**:
  - `alert_mask` - Bitmask of `AlertFlag` values (e.g. `SHUNT_OVER_VOLTAGE`).
  - `alert_limit` - Raw threshold value for the ALERT_LIMIT register.
- **Returns**: `ESP_OK` on success, or `ESP_ERR_*` on I2C failure.

#### `virtual esp_err_t is_conversion_ready(bool& out_ready) = 0`
Checks the conversion-ready flag (CVRF) in the MASK_ENABLE register. CVRF (bit 3) is set when a new conversion completes and is cleared when MASK_ENABLE is read (datasheet SBOS448B).
- **Parameters**: `out_ready` - Set to true when a new measurement is available.
- **Returns**: `ESP_OK` on success, or `ESP_ERR_*` on I2C failure.

#### `virtual const Ina226Config& get_config() const = 0`
Returns the currently active configuration.
- **Returns**: Reference to the active `Ina226Config`.

---

### `Ina226Driver` (Concrete Implementation)
Defined in `ina226_driver/include/ina226_driver.hpp`. Implements `IIna226Driver` on top of `idf_hals::II2cHAL`.

#### `Ina226Driver(idf_hals::II2cHAL& i2c_hal, const Ina226Config& config = Ina226Config())`
Creates the driver bound to an I2C HAL implementation and a configuration. The constructor does not touch the hardware; call `init()` (or `init(bus_handle)`) before performing any read/write.
- **Parameters**:
  - `i2c_hal` - I2C HAL implementation (dependency injection, enables mocking in tests).
  - `config` - Driver configuration (address, shunt, calibration, conversion settings).

#### `esp_err_t read_register(Register reg, uint16_t& out_val)`
Low-level I2C read of a 16-bit register. Not part of the `IIna226Driver` interface; exposed for testing and direct access.
- **Parameters**:
  - `reg` - Register address to read (see `Register` enum).
  - `out_val` - 16-bit register value.
- **Returns**: `ESP_OK` on success, or `ESP_ERR_*` on I2C failure.

#### `esp_err_t write_register(Register reg, uint16_t val)`
Low-level I2C write of a 16-bit register. Not part of the `IIna226Driver` interface; exposed for testing and direct access.
- **Parameters**:
  - `reg` - Register address to write (see `Register` enum).
  - `val` - 16-bit value to write.
- **Returns**: `ESP_OK` on success, or `ESP_ERR_*` on I2C failure.
