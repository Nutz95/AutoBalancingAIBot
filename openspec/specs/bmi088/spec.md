# bmi088 Specification

## Purpose
TBD - created by archiving change add-bmi088-reader. Update Purpose after archive.
## Requirements
### Requirement: BMI088 sensor reader
The system SHALL provide a BMI088 driver that can be initialized and polled to return accelerometer and gyroscope samples as a timestamped struct.

#### Scenario: Read sample
- **WHEN** the driver is initialized and `read()` is called at 200Hz
- **THEN** the driver returns gyro (rad/s) and accel (m/s^2) samples with a timestamp

### Requirement: Configuration storage
The system SHALL provide a `BMI088Config` type to hold sensor configuration including CS pins and target sampling frequency in Hz.

#### Scenario: Config available
- **WHEN** a component needs to read the sensor
- **THEN** it can obtain sampling settings from `BMI088Config` and use them to determine read scheduling

