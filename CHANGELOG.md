# Changelog

All notable changes to the `ina226_driver` component will be documented in this file.

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
