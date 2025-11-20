# Change: add-bmi088-reader

## Why
Provide a small, testable BMI088 driver and configuration class for the ESP32-S3 PlatformIO project so the firmware can read gyro and accelerometer data at a fixed frequency (200Hz). This creates a clear abstraction for the sensor and centralizes configuration values for later reuse.

## What Changes
- Add a capability spec delta describing the BMI088 reader requirement
- Implement a `BMI088Config` configuration class to hold sensor configuration (CS pins, sampling frequency)
- Implement a `BMI088Driver` C++ class that wraps the `bmi088-arduino` library and exposes a simple `begin()` and `read()` interface
- Update `ESP32/platformio.ini` to include the BMI088 Arduino library as a project dependency

## Impact
- New files under `ESP32/include/` and `ESP32/src/` (driver + config)
- A new change proposal under `openspec/changes/add-bmi088-reader/`

## Acceptance Criteria
- `openspec validate add-bmi088-reader --strict` passes
- Project compiles (`pio run`) after adding driver and dependency
- The `BMI088Driver` exposes a method to read accel+gyro samples and returns timestamps
