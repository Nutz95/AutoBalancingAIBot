# bmi088 spec delta

## ADDED Requirements

### Requirement: BMI088 configuration defaults

The system SHALL provide correct default chip-select pin assignments for the BMI088 accelerometer and gyroscope in `BMI088Config`.

#### Scenario: Correct default CS pins

- **GIVEN** the project scaffold and default `BMI088Config` values
- **WHEN** a consumer initializes the driver without overriding CS pins
- **THEN** the accelerometer chip-select defaults to GPIO4 and the gyroscope chip-select defaults to GPIO14

#### Notes and hardware variations

- The driver performs WHO_AM_I probes at startup and expects the accelerometer ID `0x1E` and the gyroscope ID `0x0F` by default.
- Some vendor modules may return an alternative gyro ID (observed `0x22` in a module); the runtime diagnostics report observed IDs to help triage.
- The driver uses `SPI_MODE0` for accel accesses and `SPI_MODE3` for gyro accesses, and performs a dummy SPI transfer before reading registers to match library semantics.
