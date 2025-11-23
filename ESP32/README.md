# AutoBalancingAIBot — ESP32 Starter

This folder contains a minimal PlatformIO starter project for the ESP32-S3 used by the AutoBalancingAIBot project.

Build

Open a terminal in this folder and run:

```powershell
pio run
```

Upload (after configuring `upload_port` in `platformio.ini` if needed):

```powershell
pio run --target upload
```

Serial monitor:

```powershell
pio device monitor --baud 115200
```

This project prints a heartbeat on serial every second.

## Calibration (IMU)

This project includes a simple IMU calibration utility to capture and persist gyroscope bias and a single-position accelerometer offset. Use the serial monitor at `921600` (or the project default) to run the commands below.

Commands (sent as plain text lines):

- `CALIB START GYRO`  — quick gyro bias calibration (samples N measurements and saves `gyro_bias`).
- `CALIB START ACCEL` — accelerometer single-position calibration (place device in the desired orientation, typically flat Z-up).
- You can optionally supply the sample count N, e.g. `CALIB START GYRO 1000` or `CALIB START ACCEL 2000`.
- `CALIB DUMP`        — print stored calibration parameters.
- `CALIB RESET`       — erase stored calibration.

Notes and recommended defaults:

- Default sample count is 2000 — this is a compromise that gives better precision (≈10s at 200Hz). For faster but less precise results you can reduce to 500–1000 samples. The calibration routines use an online algorithm (no large heap allocations).
- During calibration verbose debug logs are suppressed to avoid flooding the serial console; only progress and results are printed.
- Ensure the robot is stationary during sampling; the procedure checks stability (variance) and will produce noisy results if the device moves.

Stability checks:

- The calibration measures the sample standard deviation during acquisition and will abort if readings are unstable.
- Default thresholds: gyro std &gt; 0.02 rad/s (~1.15 deg/s) aborts; accel std &gt; 0.2 m/s^2 aborts.

Procedure example:

1. Secure the robot so wheels cannot move.
2. Open the serial monitor at `921600`.
3. Send `CALIB START GYRO` and wait for `CALIB DONE` with the printed bias values.
4. Place the robot in the single reference orientation (flat, Z-up) and send `CALIB START ACCEL`.
5. Send `CALIB DUMP` to confirm the saved values.
6. Reboot and verify calibration is loaded automatically.

## Motor driver (SCServo)

This project includes a minimal motor driver integrating with the `SCServo`/STS family of servos.
It provides a small serial UI for bench testing and safe operation. Motors are disabled by default at boot
and must be enabled explicitly.

Basic serial commands (send as plain text lines):

- `MOTOR ENABLE`          - initialize servo bus (on-demand) and enable torque
- `MOTOR DISABLE`         - disable torque (safe state)
- `MOTOR STATUS`          - print motor enabled state and IDs
- `MOTOR DUMP`            - print motor config (IDs, pins)
- `MOTOR READ <LEFT|RIGHT|ID>` - read encoder/position from servo
- `MOTOR SET <LEFT|RIGHT|ID> <v>` - set normalized command in [-1.0..1.0]
	- Normalized values are mapped to servo units using `SC_SERVO_MAX_SPEED` (default 7000). Start with small values (e.g. `0.05`).
- `MOTOR SET <LEFT|RIGHT|ID> RAW <value>` - send raw signed servo units directly (use short pulses)
- `MOTOR PARAMS <LEFT|RIGHT|ID>` - dump servo EEPROM/SRAM parameters and present status

Safety notes:
- Keep wheels mechanically restrained during bench testing.
- Start with low speeds and short pulses for `RAW` commands.
- `LEFT_MOTOR_INVERT` and `RIGHT_MOTOR_INVERT` are available in `ESP32/config/motor_config.h` to correct direction.


