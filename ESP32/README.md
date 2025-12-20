# AutoBalancingAIBot — ESP32 Starter

This folder contains a minimal PlatformIO starter project for the ESP32-S3 used by the AutoBalancingAIBot project.

## PlatformIO Installation & Setup

### Option 1: VS Code Extension (GUI only)
Install the PlatformIO IDE extension from the VS Code marketplace. This provides a graphical interface and integrated build commands but does not add the `pio` CLI to your system PATH.

### Option 2: CLI Installation (Recommended for terminal usage)
To use `pio` commands in PowerShell, MSYS2, or other terminals, install PlatformIO CLI via `pipx`:

```powershell
python -m pip install --user pipx
python -m pipx ensurepath
```

Close and reopen your terminal, then install PlatformIO:

```powershell
pipx install platformio
```

**PATH Configuration (if needed):**
If `pio --version` fails after installation, you may need to manually add the pipx Scripts directory to your system PATH:
- Typical path: `C:\Users\<YourUsername>\pipx\venvs\platformio\Scripts`
- Add this to your user PATH environment variable in Windows Settings → System → Advanced system settings → Environment Variables
- Restart VS Code and terminals for the PATH change to take effect

Verify installation:
```powershell
pio --version
```

Alternative: Use Python module invocation without PATH modification:
```powershell
python -m platformio run
```

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

Build / Helper scripts
----------------------

This project provides small helper scripts (PowerShell and MSYS2 bash) in the project root to run PlatformIO commands with consistent messaging.

PowerShell helpers (Windows):

- `build_firmware.ps1` — runs `pio run` in the `ESP32` folder to build the firmware.
- `build_and_upload.ps1` — runs `pio run` then `pio run --target upload` to build and upload in one step.
- `upload_firmware.ps1` — runs `pio run --target upload` to upload a previously built firmware.

Bash helpers (MSYS2 / MinGW64 / WSL):

- `build_firmware.sh` — runs `pio run` in the `ESP32` folder to build the firmware.
- `build_and_upload.sh` — runs `pio run` then `pio run --target upload` to build and upload in one step.
- `upload_firmware.sh` — runs `pio run --target upload` (for upload-only flows).

Usage (bash / MSYS2):

```bash
# From the ESP32 project folder
./build_firmware.sh            # build only
./build_and_upload.sh          # build then upload
./upload_firmware.sh           # upload only (assumes build already done)
```

Notes:
- The scripts detect whether `pio` (PlatformIO CLI) is available and will attempt `platformio` or `python -m platformio` if needed.
- You can pass additional PlatformIO arguments to the scripts; they will be forwarded to `pio` (for example `-e <env>`).
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
 	- Example: `MOTOR ACC LEFT 200` (default smoothing; repository default value is set in `ESP32/config/motor_configs/servo_motor_config.h` as `MOTOR_SERVO_DEFAULT_ACC`)
- `MOTOR SET <LEFT|RIGHT|ID> RAW <value>` - send raw signed servo units directly (use short pulses)
- `MOTOR PARAMS <LEFT|RIGHT|ID>` - dump servo EEPROM/SRAM parameters and present status

Notes on alternative motorizations
---------------------------------

 - DC motors with Hall-effect encoder: supported hardware example — JGB37-520 (12V) with 56:1 gearbox.
	- These motors include a dual Hall-effect encoder (two sensors). Vendor wiring and signal mapping (translated from the product documentation):
		- Red wire — motor positive supply (+). Reversing motor + and - changes rotation direction.
		- White wire — motor negative supply (-).
		- Yellow wire — encoder signal A (vendor notes ~11 pulses per motor revolution).
		- Green wire — encoder signal B (vendor notes ~11 pulses per motor revolution from the second sensor).
		- Blue wire — encoder supply positive for the sensors (+3.3–5 V). Do not swap polarity.
		- Black wire — encoder supply negative (GND). Do not swap polarity.
	- Encoder resolution: the vendor specifies ~11 pulses per motor revolution per sensor; in quadrature this corresponds to approximately 44 counts per motor revolution. This value is measured at the motor shaft (before the gearbox reduction). To obtain counts per output-shaft revolution, multiply by the gearbox ratio (or divide accordingly depending on how you convert motor→output shaft).
	- Mechanical details: output shaft diameter 6 mm (D-type), length 14 mm.
	- The vendor motor specification table (speeds, no-load currents, stall torque, gearbox lengths for each reduction) is included in this repository under `ESP32/tools/JGB37-520_Table.png` — consult it for exact numbers matching your gearbox choice.
	- These motors require an H-bridge motor driver with sufficient continuous/peak current capability and an encoder reader on the MCU to implement closed-loop control (velocity/position).

 - DC motor driver example: BTS7960 (43A)
	- The project commonly uses the BTS7960 dual-MOSFET H-bridge modules (marketed as "BTS7960 43A" motor drivers). Typical characteristics to consider:
		- Dual half-bridge power stage suitable for 6–24 V motor supplies (verify module rating and power connectors for your variant).
		- Peak current rating often advertised around 43 A (short bursts); sustained/continuous current depends on cooling and the exact module — expect much lower continuous ratings without significant heatsinking (e.g., 10–30 A depending on cooling).
		- Typical control interface: PWM input(s) for direction/speed, enable signals, and gate/logic powered at 5 V (check your module). Some boards provide current-sense outputs, but many low-cost modules do not — use appropriate fusing and current measurement if needed.
		- Provide proper heat-sinking, wiring, and fusing — high-current operation requires careful thermal and supply design.
	- When using BTS7960 modules with the JGB37 motors, ensure your power supply can supply the motor stall/transient currents and include a fuse or current-limiter for safety.
	- Vendor / module features (typical IBT-2 / BTS7960 modules):
		- Uses Infineon BTS7960 high-power H-bridge driver chips with onboard protections.
		- Over-temperature, over-voltage, under-voltage and over-current protection.
		- Short-circuit protection.
		- Current diagnostics/support and slope (current ramp) adjustment.
		- Dead-time generation between half-bridges for safe switching.
		- Typical module designation: IBT-2.
		- Input voltage range commonly 6 V–27 V (check your module label).
		- Maximum (peak) current advertised: up to 43 A (short bursts). Continuous current depends on cooling and wiring.
		- Logic/input level: 3.3–5 V (PWM or level control). Duty cycle: 0–100%.
	- These features mean BTS7960 modules are a robust choice for medium-power DC motors, but verify your exact module's datasheet and observe safe wiring, cooling, and fusing practices when using them with the JGB37 family.

- Stepper motors (NEMA17) with step driver / MKS SERVO42D: the project can be adapted to drive steppers using `EN/DIR/STEP` pins and typical stepper driver wiring. Steppers are suited when precise open-loop positioning or microstepping velocity control is desired.

STS3215HS servo evaluation
-------------------------

An initial attempt to use the STS3215HS / SCServo family for velocity-mode balancing was _not_ successful in this project. In practice we observed a significant latency between issuing a velocity command and the servo's physical response in Wheel/velocity mode. That latency (and internal smoothing/acceleration behavior) made closed-loop balancing unreliable. For that reason the codebase now includes a modular `motor_drivers/` layout so you can implement and test alternate drivers (DC+encoder, stepper) more easily.

Filter selection & per-filter PID gains
-------------------------------------

- `FILTER LIST` — list available IMU/fusion filters (e.g. `MADGWICK`, `COMPLEMENTARY1D`, `KALMAN1D`).
- `FILTER STATUS` — show the currently active filter.
- `FILTER SELECT <name>` — switch the runtime filter to `<name>` and persist that choice across reboots.

The balancer PID gains are persisted per active filter. Use `BALANCE GAINS <kp> <ki> <kd>` to set gains — the values will be saved in NVS under keys namespaced by the active filter, for example `MADGWICK_bp`, `MADGWICK_bi`, `MADGWICK_bd` (when `MADGWICK` is active). When you switch filters the controller will load the gains associated with that filter.

Safety notes:
- Keep wheels mechanically restrained during bench testing.
- Start with low speeds and short pulses for `RAW` commands.
- `LEFT_MOTOR_INVERT` and `RIGHT_MOTOR_INVERT` are available in `ESP32/config/motor_configs/servo_motor_config.h` to correct direction (for servo driver). For DC drivers see `ESP32/config/motor_configs/dc_motor_config.h`.

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

Agents workflow (repository)
--------------------------------
This repository includes optional VS Code Custom Agents to help plan, generate and verify
code changes for the ESP32 firmware. Agent configs are in `.github/agents/`.

- `Plan` (existing): produces a structured plan for requested changes.
- `Coder`: generates C++ code following `CODING_RULES.md` and SOLID principles.
- `Verifier`: checks SOLID, coding rules and documentation updates.

If you use the VS Code Copilot/Custom Agents feature, place agent configs in `.github/agents/`
or configure them via the extension UI. For automated builds/tests use the MSYS2 MinGW64 shell
or PlatformIO as described above.

Wi‑Fi console (TCP)
-------------------

This firmware optionally includes a small TCP "console" server that forwards
device logs and accepts textual commands over the network. Enable it by
setting `WIFI_CONSOLE_ENABLED 1` in `ESP32/config/board_config.h` or by
customizing your board header.

Behavior summary
- On boot the firmware reads NVS keys `wifi_ssid` and `wifi_pass`. If both
	values are present the device will attempt to connect automatically to the
	configured network and, once connected, start a TCP console server (default
	port `2333`).
- The console forwards the device log lines and accepts plain-text command
	lines (the same commands available on the USB serial console).

Compile-time flag
-----------------

The Wi‑Fi console build can be enabled or disabled at compile-time using the
`WIFI_CONSOLE_ENABLED` flag in `ESP32/config/board_config.h`. When set to
`1` the console module is compiled in and the device will attempt to connect
and start the TCP server on boot (when credentials are present). Setting the
flag to `0` removes the console from the build and reduces firmware size and
runtime resource usage.

Configure Wi‑Fi via USB serial (recommended for first setup)
----------------------------------------------------------

1. Connect the board over USB and open the serial monitor (default baud as in
	 project config).
2. Enter Wi‑Fi SSID:

	 WIFI SET SSID MyNetworkSSID

3. Enter Wi‑Fi password:

	 WIFI SET PASS MySecretPassword

4. Optional: verify the saved values and current connection status:

	 WIFI STATUS

	 WIFI DIAG

Notes:
- The commands persist values to NVS (non-encrypted). Use these commands to
	change credentials when needed. The device will attempt to connect
	automatically on subsequent boots if credentials are present.
- If you change credentials and want the connection to take effect immediately
	you can reboot or use any provided `WIFI CONNECT` action in the serial menu
	(if implemented).

Using the Python client (included)
---------------------------------

An example client lives at `ESP32/tools/wifi_console_client.py`.

Run it from the `ESP32` folder like this (PowerShell):

```powershell
python .\tools\wifi_console_client.py <ESP_IP> [2333]
```

Features of the provided client:
- Enables TCP keepalive on the socket to help the OS detect dead peers.
- Implements an automatic reconnect loop with exponential backoff when the
	connection is lost.
- On a send failure (for example because the ESP was reflashed during a send)
	the client remembers the failed command and will attempt to resend it after
	a successful reconnect. The client prints clear status messages such as
	`[Disconnected]`, `[Reconnecting]`, and `[Resent pending command]`.

Usage notes and testing guidance
--------------------------------
- Start the client and connect to the ESP IP. You should see log lines from
	the device appear in your terminal.
- If you flash or reboot the ESP while the client is connected, the client
	will detect the disconnection and try to reconnect automatically. If you
	typed a command at the moment the connection failed, the client will attempt
	to resend that command once the connection is restored.
- Example test flow:
	1. Start the Python client and connect.
	2. While connected, issue a command (for example `?` or `HELP`) and verify
		 the response.
	3. Reflash the ESP (or reboot it). The client should print `[Disconnected]`
		 followed by reconnect attempts and then `[Resent pending command]` if a
		 command needed resending.

Security note
-------------
- Wi‑Fi credentials are persisted in NVS in plaintext; this is more convenient
	than hardcoding but not encrypted—use the TCP console only on trusted
	networks or add authentication when exposing the device to untrusted
	environments.

Notes
-----
- The TCP console is intended for tuning and debugging during development.
- The existing BLE HID code continues to work when the TCP console is active.
- If you prefer not to enable Wi‑Fi, continue to use the USB serial connection
	for logs and command input.


