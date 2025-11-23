# Design: SCServo motor driver

This document records the minimal design decisions for the SCServo-based motor driver.

1. Configuration
   - Motor IDs and direction inversion live in a tiny header `ESP32/config/motor_config.h`. This avoids adding new runtime JSON parsing and is easy to edit in embedded builds.
   - Defaults: LEFT_MOTOR_ID = 8, RIGHT_MOTOR_ID = 7, LEFT_INVERT = true (or set by user if wiring reversed).

2. API
   - `enableMotors()` / `disableMotors()` toggle a module-level flag. When disabled, `setMotorCommand()` must return early.
   - `areMotorsEnabled()` returns the flag.
   - `setMotorCommand(int id, float cmd)` accepts a normalized command in range [-1.0, +1.0] where sign indicates direction. The driver maps this to SCServo position/velocity commands and applies per-motor inversion.
   - `readEncoder(int id)` returns an integer position or 0 for stub mode.

3. SCServo integration notes
   - Use the `workloads/SCServo` library for servo communication (UART/serial). Initialize the bus at boot in `motor_driver.cpp` when real mode is enabled.
   - Provide a compile-time flag (`MOTOR_DRIVER_REAL=1`) to switch between stub and real modes.

4. Safety
   - Motors default to disabled at boot.
   - Provide logging for every enable/disable/setCommand call in stub mode to help bench testing.
