# Changelog

All notable changes to the `ina226_driver` component will be documented in this file.

## [0.2.0] - 2026-08-10

### Added
- `IIna226Driver::read_alert_flags()` to read and clear the MASK_ENABLE flags. Because AFF/CVRF are read-to-clear (datasheet SBOS448B), it deasserts the ALERT pin and re-arms the next alert — the required acknowledgement when using `ALERT_ON_CONVERSION_READY` (CNVR), replacing the previous manual `read_register(MASK_ENABLE)` pattern.
- `Ina226Driver::is_conversion_ready()` now internally uses `read_alert_flags()` (behavior unchanged: still reads MASK_ENABLE and reports the CVRF bit).

## [0.1.0] - 2026-08-10

### Added
- Initial implementation of the standalone C++ `ina226_driver` component.
- Dependency injection pattern: abstract `IIna226Driver` interface and I2C hardware abstraction via `idf_hals`.
- Chip identification verification (manufacturer ID `0x5449` and die ID `0x2260`) during initialization.
- Bidirectional shunt voltage, bus voltage, current, and power measurement with datasheet-correct LSB scaling.
- Calibration validation ensuring the CAL value fits the 15 writeable bits of the CALIBRATION register (max `0x7FFF`).
- Self-clearing RST bit polling on soft reset (bounded loop, RTOS-agnostic, no task delays).
- Alert configuration via ALERT_LIMIT and MASK_ENABLE bitmasks, including conversion-ready flag (CVRF) polling.
- Runtime reconfiguration through `set_config()`.
- Comprehensive GoogleTest suite executing on Linux host.
- Independent GitHub Actions build and host test CI workflows.
