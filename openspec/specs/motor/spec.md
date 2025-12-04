# motor Specification

## Purpose
Provide the motor driver specification and expected runtime behaviour for the
ESP32 SCServo-based motor stack. This spec documents the public motor API used
by the balancer and input subsystems (gamepad/Bluetooth), the compile-time
configuration headers used to set IDs and per-motor inversion, and the safety
defaults (motors disabled at boot). This spec was created when archiving the
`add-motor-driver-scservo` change on 2025-12-04.
## Requirements
### Requirement: Motor driver API

- The system SHALL provide a motor driver module exposing the following functions: `enableMotors()`, `disableMotors()`, `areMotorsEnabled()`, `setMotorCommand(int id, float cmd)`, `setMotorCommandBoth(float left_cmd, float right_cmd)`, `setMotorCommandRaw(int id, int16_t rawSpeed)`, and `readEncoder(int id)`.

#### Scenario: Stubs compile and log

- Given the project is built with stub mode (compile-time `MOTOR_DRIVER_REAL=0`), when the firmware boots, then motors are disabled by default and calls to `enableMotors()`/`disableMotors()`/`setMotorCommand()` produce informative serial logs and do not drive hardware. In stub mode `readEncoder()` returns a deterministic value (typically `0`).

### Requirement: Configuration

- The system SHALL allow configuring motor IDs and per-motor inversion (left/right) via the compile-time header `ESP32/config/motor_config.h` (macros: `LEFT_MOTOR_ID`, `RIGHT_MOTOR_ID`, `LEFT_MOTOR_INVERT`, `RIGHT_MOTOR_INVERT`).

#### Scenario: Inversion configuration

- Given the driver is configured with `LEFT_MOTOR_INVERT=1` and motor commands are symmetric, when `setMotorCommand(left_id, +0.5)` and `setMotorCommand(right_id, +0.5)` are invoked, then the actual applied directions reflect the inversion config (left inverted). The header `ESP32/config/motor_config.h` is authoritative for default IDs and inversion flags.

### Requirement: Safety default

- The driver SHALL default to motors disabled at boot and ignore `setMotorCommand()` until `enableMotors()` is explicitly called. `enableMotors()`/`disableMotors()` toggle the module-level enabled flag; `areMotorsEnabled()` returns it.

#### Scenario: Bench test safety

- Given the motors are connected and wheels are restrained, when `enableMotors()` is called for the first time on a bench, then logs indicate motors enabled and subsequent `setMotorCommand()` calls map normalized commands to SCServo velocity/position frames only if `MOTOR_DRIVER_REAL` is enabled. `setMotorCommand(int id, float cmd)` accepts a normalized float in the range [-1.0, +1.0] where sign indicates direction; `setMotorCommandRaw(int id, int16_t rawSpeed)` accepts the servo-native raw speed units.

