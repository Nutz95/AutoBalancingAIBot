# Change proposal: add-motor-driver-scservo

## Summary

Implement a minimal SCServo-based motor driver for the AutoBalancingAIBot. The driver will provide a safe, testable API to enable/disable motors, send motor commands, and read encoder/feedback from STS3215HS servos via the `SCServo` library. Default IDs: left motor = 8, right motor = 7. Because the motors face each other (mirrored), the driver must support per-motor direction inversion via configuration.

## Motivation

To progress from sensor calibration to closed-loop balancing we need a safe motor interface. A small, well-scoped driver will allow bench testing (stubs first) and later full integration with `workloads/SCServo` for live robot tests.

## What changes

- Add a motor driver module exposing `enableMotors()`, `disableMotors()`, `areMotorsEnabled()`, `setMotorCommand(id,cmd)`, and `readEncoder(id)`.
- Use the `SCServo` library for communication and feedback.
- Add a small configuration file / header to declare motor IDs and inversion flags.
- Provide safe defaults (motors disabled at boot) and unit-test-friendly stubs for bench testing.

## Scope

- Files to add/modify:
  - `ESP32/include/motor_driver.h` (new)
  - `ESP32/src/motor_driver.cpp` (new)
  - `ESP32/config/motor_config.h` or similar (new) for ID/inversion settings
  - Update docs: `ESP32/README.md` (short snippet) and `ESP32/TODO_BALANCER.md` (task progress)

## Acceptance criteria

- The project builds with the new driver stubs.
- `disableMotors()` prevents `setMotorCommand()` from having any effect; calls are logged when running in stub mode.
- Configuration allows setting left/right IDs and per-motor inversion.
- Integration plan for `workloads/SCServo` is documented.

## Risks & mitigations

- Risk: sending commands to motors with wheels free — mitigation: default to motors disabled at boot and require explicit `enableMotors()` after safety checks.
- Risk: incorrect direction mapping — mitigation: per-motor inversion config and bench tests with wheels restrained.

## Rollout

- Stage 1: add stubs + config + docs and validate build (this change)
- Stage 2: swap stubs for real `SCServo` calls and test on bench with wheels restrained
- Stage 3: add encoder telemetry and integrate with balance controller
