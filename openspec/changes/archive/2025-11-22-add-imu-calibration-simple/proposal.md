# Change proposal: add-imu-calibration-simple

## Summary

Add a simple IMU calibration routine (quick mode) to compute and store the gyroscope bias and accelerometer offsets. Provide a minimal serial interface to start calibration and display results. Temporarily reduce or disable verbose debug logging on the serial link during calibration to avoid flooding the console.

## Motivation

The project now reliably reads BMI088 sensor data, but without bias and offset compensation. Without calibration the balancing controller will suffer from drift and estimation errors that complicate stabilization. A simple calibration at startup improves immediate accuracy and controller stability.

## Why

Providing a simple, repeatable method to measure and compensate gyro bias and accel offsets yields more stable angle estimates at boot, reduces manual tuning, and enables more reliable control experiments.

## What Changes

- Add an `imu_calibration` module that:
  - computes `gyro_bias` as the mean of N samples (quick mode);
  - computes an `accel_offset` for the tested static position (single-position calibration);
  - persists calibration parameters to NVS/EEPROM and reloads them at boot;
  - temporarily suppresses verbose serial debug during calibration;
  - exposes a minimal serial UI to start/monitor calibration (`CALIB START GYRO`, `CALIB START ACCEL`, `CALIB DUMP`, `CALIB RESET`).

- Add manual and smoke tests to verify parameters are saved and reapplied after reboot.

## Scope

- Affected/added files:
  - `ESP32/src/imu_calibration.cpp` (new)
  - `ESP32/include/imu_calibration.h` (new)
  - `ESP32/src/main.cpp` or the IMU init entry point: call `load_calibration()` at startup
  - `ESP32/src/BMI088Driver.cpp`: call `apply_calibration()` on raw reads
  - Temporarily reduce serial debug verbosity while calibration runs (coordinate with the global logger)

- Not included: full multi-position (6-position) calibration and temperature compensation (may be added later).

## Acceptance criteria

- `CALIB START GYRO` triggers sampling of N measurements and prints `CALIB DONE` with `gyro_bias` values.
- `CALIB START ACCEL` samples the current position and prints `CALIB DONE` with `accel_offset` values.
- `CALIB DUMP` returns stored parameters; `CALIB RESET` erases calibration from NVS.
- Parameters are persisted and reloaded at boot; `apply_calibration()` corrects IMU readings used by the angle filter.
- During calibration verbose debug logs are suppressed on the serial link (only progress/errors printed).

## Risks & Mitigations

- Risk: user triggers calibration while motors are active → mitigation: disable motors during calibration or refuse calibration if motors enabled.
- Risk: hardware movement during sampling → mitigation: check sample variance/std and require device to be stable before accepting samples.

## Rollout

- Implement quick mode first (GYRO + single-position ACCEL). Validate on bench; add multi-position option later if needed.
