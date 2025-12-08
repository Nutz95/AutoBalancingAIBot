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

PowerShell helper scripts
-------------------------

For convenience this project includes small PowerShell helpers at the project root to run PlatformIO commands with consistent messaging:

- `build_firmware.ps1` — runs `pio run` in the `ESP32` folder to build the firmware.
- `build_and_upload.ps1` — runs `pio run` then `pio run --target upload` to build and upload in one step.
- `upload_firmware.ps1` — runs `pio run --target upload` to upload a previously built firmware.

Usage (PowerShell):

```powershell
# From the ESP32 project folder
.\build_firmware.ps1            # build only
.\build_and_upload.ps1         # build then upload
.\upload_firmware.ps1          # upload only (assumes build already done)
```

Notes:
- The scripts detect whether `pio` (PlatformIO CLI) is available and print helpful errors if not.
- You can pass additional PlatformIO arguments to the scripts, they will be forwarded to `pio` (for example `-e <env>`).
- Configure `upload_port` in `platformio.ini` or pass upload options to the upload script if your board is on a non-standard port.


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
 - `MOTOR SET <LEFT|RIGHT|ID> <v>` - set normalized command in [-1.0..1.0]
 	- Normalized values are mapped to servo units using `SC_SERVO_MAX_SPEED` (default 7000). Start with small values (e.g. `0.05`).
 - `MOTOR ACC <LEFT|RIGHT|ID> <value>` - set servo acceleration register at runtime
 	- Example: `MOTOR ACC LEFT 0` (no smoothing, fastest changes)
 	- Example: `MOTOR ACC LEFT 200` (default smoothing; repository default value is set in `ESP32/config/motor_config.h` as `MOTOR_SERVO_DEFAULT_ACC`)
- `MOTOR SET <LEFT|RIGHT|ID> RAW <value>` - send raw signed servo units directly (use short pulses)
- `MOTOR PARAMS <LEFT|RIGHT|ID>` - dump servo EEPROM/SRAM parameters and present status

Filter selection & per-filter PID gains
-------------------------------------

- `FILTER LIST` — list available IMU/fusion filters (e.g. `MADGWICK`, `COMPLEMENTARY1D`, `KALMAN1D`).
- `FILTER STATUS` — show the currently active filter.
- `FILTER SELECT <name>` — switch the runtime filter to `<name>` and persist that choice across reboots.

The balancer PID gains are persisted per active filter. Use `BALANCE GAINS <kp> <ki> <kd>` to set gains — the values will be saved in NVS under keys namespaced by the active filter, for example `MADGWICK_bp`, `MADGWICK_bi`, `MADGWICK_bd` (when `MADGWICK` is active). When you switch filters the controller will load the gains associated with that filter.

Safety notes:
- Keep wheels mechanically restrained during bench testing.
- Start with low speeds and short pulses for `RAW` commands.
- `LEFT_MOTOR_INVERT` and `RIGHT_MOTOR_INVERT` are available in `ESP32/config/motor_config.h` to correct direction.

Testing on host (MSYS2 / PowerShell)
-----------------------------------

The project now provides a comprehensive host-side test suite for the IMU fusion algorithm in
`ESP32/test/imu_fusion_tests.cpp`. This suite covers the original lightweight unit test plus
additional robustness and fuzz tests.

Follow `ESP32/test/README_MSYS2.md` for MSYS2-based build instructions. You can run the provided
PowerShell helper to compile and run the full test suite:

```powershell
cd 'I:\GIT\AutoBalancingAIBot\ESP32\test'
.\build_test.ps1
```

By default the test binary executes two groups for the magnetometer-dependent cases:
- a non-strict pass that prints "EXPECTED FAIL (no magnetometer)" (does not count as a failure),
- a strict pass that is intended to be run when a magnetometer (or a simulated heading) is available
	— this strict phase is skipped by default for robots without a magnetometer.

To change strict behavior see `ESP32/test/imu_fusion_tests.cpp` (there is a `RUN_STRICT_MAG`
toggle near the top of the file) or use the compile-time macro `-DRUN_STRICT_MAG=1` when
compiling manually.

## Interactive Serial Menu (numeric)

This project includes a small interactive numeric serial menu used by the `HELP` command.

- Location: `ESP32/src/serial_menu.cpp` and `ESP32/include/serial_menu.h`.
- Behavior: type `HELP` (or `?`) in the serial monitor to enter the menu. Use numeric choices
	to navigate and run commands. `0` goes back or exits the menu.
- Pattern: the menu is a small registration-style builder — entries and submenus are added
	by calling `addEntry(id, label, handler)` or `addSubmenu(id, label, submenu)` where handlers
	are `std::function<void(const String& param)>` that receive the remainder of the input line
	as a parameter (so `5 LEFT 0.05` will call the handler for entry `5` with parameter
	`LEFT 0.05`). The root menu is constructed lazily on first use in `serial_commands.cpp`.

This keeps interactive flows simple while reusing the existing serial command handlers
for the actual operations (calibration, motor control, tuning).

Status LED
----------
The board exposes a single RGB status LED (NeoPixel/WS2812) when configured in
`board_config.h`. The firmware uses the LED to communicate basic system state:

- **Red**: IMU fusion warmup in progress (place the robot upright and wait).
- **Green**: Fusion is ready and gyro bias is initialized — safe to use `BALANCE START`.
- **Blinking Red**: Fatal IMU initialization error (for example, gyro CS wiring issue).

If the LED is not present on your board the functions are no-ops and the
firmware falls back to serial logging for all messages.


