# INA226 Driver Component

[![ESP-IDF Build](https://github.com/aluiziotomazelli/ina226_driver/actions/workflows/build.yml/badge.svg)](https://github.com/aluiziotomazelli/ina226_driver/actions/workflows/build.yml)
[![Host Tests](https://github.com/aluiziotomazelli/ina226_driver/actions/workflows/host_test.yml/badge.svg)](https://github.com/aluiziotomazelli/ina226_driver/actions/workflows/host_test.yml)
[![Coverage](https://img.shields.io/badge/coverage-report-blue)](https://aluiziotomazelli.github.io/ina226_driver/index.html)

A lightweight, modular, and dependency-injected C++ driver for the **Texas Instruments INA226** bi-directional current/power monitor, targeting **ESP-IDF v5.1+**.

## Features

- **Dependency Injection**: Pure virtual `IIna226Driver` interface with constructor injection, plus an I2C hardware abstraction layer (`idf_hals`) for easy unit testing and mocking.
- **Chip ID Verification**: `init()` validates the manufacturer ID (`0x5449`, "TI") and die ID (`0x2260`) before applying any configuration.
- **Bidirectional Measurements**: Reads shunt voltage (uV), bus voltage (mV), current (mA), and power (mW) with datasheet-correct LSB scaling.
- **Calibration Validation**: Rejects calibration values that do not fit the 15 writeable bits of the CALIBRATION register (max `0x7FFF`), preventing silent truncation.
- **Self-Clearing Soft Reset**: Polls the RST bit until the device returns to its power-on state before re-applying configuration (bounded loop, RTOS-agnostic, no task delays).
- **Configurable Alerts**: Writes ALERT_LIMIT and MASK_ENABLE from `AlertFlag` bitmasks, including the conversion-ready flag (CVRF) for asynchronous data-ready polling. `read_alert_flags()` reads and clears the MASK_ENABLE flags, which is required to re-arm the ALERT pin when using CNVR.
- **Runtime Reconfiguration**: `set_config()` applies new settings on the fly without re-initializing the device.
- **Host Testing**: Includes a GoogleTest/GoogleMock suite executable on Linux host with coverage reports.

---

## Directory Structure

```text
ina226_driver/
├── CMakeLists.txt              # ESP-IDF component build system file
├── idf_component.yml           # ESP-IDF component manager manifest
├── README.md                   # Core component documentation
├── API.md                      # Detailed API references
├── CHANGELOG.md                # Version log
├── LICENSE                     # MIT License
├── include/
│   ├── ina226_driver.hpp       # Concrete Ina226Driver class
│   ├── ina226_types.hpp        # Configurations, registers, enums
│   └── interfaces/
│       └── i_ina226_driver.hpp # Abstract interface contract
├── src/
│   └── ina226_driver.cpp       # Implementation logic
├── external/
│   └── idf_hals/               # I2C HAL (git submodule)
├── host_test/
│   └── test_ina226_driver/     # Standalone unit test suite
├── mocks/                      # GoogleMock mocks used by host tests
├── test_apps/                  # Target build smoke test app
└── example/                    # ESP-IDF usage examples
    ├── alert_high_voltage/     # Deep-sleep-friendly alert example
    ├── alert_interrupt/        # ALERT pin interrupt handling
    └── polling_data/           # Periodic polling example
```

---

## Getting Started

### 1. Add the Component

The component depends on the `idf_hals` submodule. Clone with submodules:

```bash
git clone --recurse-submodules https://github.com/aluiziotomazelli/ina226_driver.git
```

Then add the component directory to your project's `CMakeLists.txt` `EXTRA_COMPONENT_DIRS` list.

### 2. Initialize I2C and Read Measurements

```cpp
#include "driver/i2c_master.h"
#include "hal_i2c.hpp"
#include "ina226_driver.hpp"

extern "C" void app_main(void)
{
    // 1. Configure and initialize I2C Master Bus
    i2c_master_bus_config_t i2c_bus_config = {};
    i2c_bus_config.i2c_port = I2C_NUM_0;
    i2c_bus_config.sda_io_num = GPIO_NUM_6;
    i2c_bus_config.scl_io_num = GPIO_NUM_7;
    i2c_bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
    i2c_bus_config.glitch_ignore_cnt = 7;
    i2c_bus_config.flags.enable_internal_pullup = true;

    idf_hals::I2cHAL i2c_hal;
    i2c_master_bus_handle_t bus_handle = nullptr;
    i2c_hal.new_master_bus(&i2c_bus_config, &bus_handle);

    // 2. Configure INA226 settings (defaults shown, adjust for your shunt)
    ina226::Ina226Config config;
    config.i2c_addr = ina226::DEFAULT_I2C_ADDR; // 0x40
    config.r_shunt_ohms = 0.1f;                 // 100 mOhm shunt
    config.max_expected_current_a = 0.8192f;    // max expected current
    config.avg_mode = ina226::AveragingMode::AVG_16;
    config.vbus_ct = ina226::ConversionTime::CT_1100US;
    config.vsh_ct = ina226::ConversionTime::CT_1100US;
    config.mode = ina226::OperatingMode::SHUNT_AND_BUS_CONTINUOUS;

    // 3. Instantiate driver and initialize sensor
    ina226::Ina226Driver driver(i2c_hal, config);
    if (driver.init(bus_handle) != ESP_OK) {
        // handle error (ID mismatch, I2C failure...)
        return;
    }

    // 4. Read measurements
    uint16_t vbus_mv = 0;
    int32_t vshunt_uv = 0;
    float current_ma = 0.0f;
    float power_mw = 0.0f;

    driver.read_bus_voltage_mv(vbus_mv);
    driver.read_shunt_voltage_uv(vshunt_uv);
    driver.read_current_ma(current_ma);
    driver.read_power_mw(power_mw);
}
```

### 3. Configure an Alert

```cpp
// Assert the ALERT pin when bus voltage exceeds 12 V (threshold is in raw units:
// bus voltage LSB = 1.25 mV, so 12000 mV / 1.25 mV = 9600).
ina226::AlertFlag mask = ina226::AlertFlag::BUS_OVER_VOLTAGE;
uint16_t alert_limit = 12000 / 1.25; // 9600
driver.configure_alert(static_cast<uint16_t>(mask), alert_limit);
```

### 4. Poll for Conversion Ready

```cpp
bool ready = false;
if (driver.is_conversion_ready(ready) == ESP_OK && ready) {
    // A new measurement is available; read current/power now.
}
```

### 5. Acknowledge an Alert (re-arm the ALERT pin)

With `ALERT_ON_CONVERSION_READY` (CNVR) the ALERT pin is asserted on every
conversion completion and stays asserted until MASK_ENABLE is read.
`read_alert_flags()` returns the flags and clears them, so the pin can re-assert
on the next conversion:

```cpp
uint16_t flags = 0;
if (driver.read_alert_flags(flags) == ESP_OK) {
    // flags contains the raw MASK_ENABLE value (e.g. CVRF bit 3 set).
}
```

---

## Examples

| Example | Description |
| --- | --- |
| `example/polling_data` | Periodically reads bus voltage, shunt voltage, current, and power. |
| `example/alert_interrupt` | Handles the ALERT pin via GPIO interrupt. |
| `example/alert_high_voltage` | Deep-sleep-friendly example using an over-voltage alert to wake the chip. |

---

## Notes

- **RTOS-agnostic core**: The driver performs blocking I2C transfers and never calls task-delay APIs, so it can be used from any thread or event context.
- **I2C speed**: The device is registered on the bus at 400 kHz (fast mode).
- **Documentation**: See [`API.md`](API.md) for the full programming interface reference.

## References

- [INA226 Datasheet (TI SBOS448B)](https://www.ti.com/document-viewer/ina226/datasheet) - register map and electrical characteristics used throughout this component.
