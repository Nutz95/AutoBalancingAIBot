# bmi088 spec delta: add IMU calibration (simple)

## ADDED Requirements

### Requirement: BMI088 calibration - simple mode

The system SHALL provide a simple IMU calibration routine which:

- Computes and stores a `gyro_bias[3]` using a configurable number of samples while the device is stationary.
- Computes and stores an `accel_offset[3]` for the tested static position (single-position calibration).
- Persists calibration parameters in non-volatile storage and reapplies them at boot.
- Provides a minimal serial UI to start calibration and dump/reset parameters.

#### Scenario: Run quick gyro calibration

- **GIVEN** the device is powered and immobile
- **WHEN** the operator issues `CALIB START GYRO` over serial
- **THEN** the device samples the gyro, computes `gyro_bias`, stores it and reports `CALIB DONE` with values
