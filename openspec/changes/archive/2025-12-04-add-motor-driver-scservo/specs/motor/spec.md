# Motor driver spec

## ADDED Requirements

### Requirement: Motor driver API

- The system SHALL provide a motor driver module exposing the following functions: `enableMotors()`, `disableMotors()`, `areMotorsEnabled()`, `setMotorCommand(int id, float cmd)`, and `readEncoder(int id)`.

#### Scenario: Stubs compile and log

- Given the project is built with stub mode, when the firmware boots, then motors are disabled by default and calls to `enableMotors()`/`disableMotors()`/`setMotorCommand()` produce informative serial logs and do not drive hardware.

### Requirement: Configuration

- The system SHALL allow configuring motor IDs and per-motor inversion (left/right) via a compile-time header `ESP32/config/motor_config.h` or equivalent.

#### Scenario: Inversion configuration

- Given the driver is configured with `LEFT_INVERT=true` and motor commands are symmetric, when `setMotorCommand(left_id, +0.5)` and `setMotorCommand(right_id, +0.5)` are invoked, then the actual applied directions reflect the inversion config (left inverted).

### Requirement: Safety default

- The driver SHALL default to motors disabled at boot and ignore `setMotorCommand()` until `enableMotors()` is explicitly called.

#### Scenario: Bench test safety

- Given the motors are connected and wheels are restrained, when `enableMotors()` is called for the first time on a bench, then logs indicate motors enabled and subsequent `setMotorCommand()` calls trigger SCServo commands only if `MOTOR_DRIVER_REAL` is enabled.
