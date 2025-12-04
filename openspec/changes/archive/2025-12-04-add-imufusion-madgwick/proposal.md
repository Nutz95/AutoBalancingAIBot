# Change: Add IMU fusion (Madgwick) for gyro+accel

## Why

To perform closed-loop self-balancing we need a robust estimate of the robot's pitch (and roll) and its rate. The calibrated BMI088 provides accelerometer and gyroscope data, but the gyroscope alone drifts with time and the accelerometer is noisy and disturbed during dynamic motion. A lightweight sensor fusion algorithm (Madgwick) produces a stable quaternion-based orientation estimate suitable for a 100–200 Hz control loop.

## What Changes

- Add an `imu_fusion` capability implementing the Madgwick algorithm (gyro + accel, no magnetometer for now).
- Provide a small C++ API for the ESP32: init, update(gx,gy,gz,ax,ay,az,dt), getters for quaternion and Euler angles (pitch, roll) and pitch rate.
- Add a CSV/TUNING mode to stream timestamp,pitch,pitch_rate,motor_cmd for PID tuning.
- Integrate `imu_fusion` with existing IMU task pipeline (200 Hz) and expose hooks for the balancing controller.
- Provide tests/integration steps and recommended `beta` defaults and tuning guidance.

## Impact

**Affected specs:**
- `imu-fusion` (new capability)
- `motor-control` (consumer of `getPitch()` / pitch_rate in control loop)

**Affected code / new files:**
- `ESP32/include/imu_fusion.h` (new) — public API
- `ESP32/src/imu_fusion.cpp` (new) — Madgwick implementation
- `ESP32/src/balance_controller.cpp` (new, later) — will consume fusion outputs (not part of this change)
- Small integration point in `ESP32/src/SystemTasks.*` or existing IMU task to call `fusion.update(...)`

**Dependencies:**
- None new (uses existing BMI088 driver outputs). No magnetometer support for this change.

**Breaking changes:**
- None. This is additive.

**Hardware scope:**
- Works with the existing BMI088 sensor. No additional hardware required.

**Acceptance criteria (high level):**
- The repo contains the `imu_fusion` implementation and spec delta.
- `openspec validate add-imufusion-madgwick --strict` passes.
- Integration plan and tuning instructions are present in `design.md`.
