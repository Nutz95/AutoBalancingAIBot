// motor_common_config.h - small set of motor settings shared across drivers
// Keep this minimal: only values that apply to all motor drivers (IDs,
// inversion flags and velocity mapping constants).
#pragma once

// motor_common_config.h - placeholder for truly shared motor settings.
// Intentionally minimal: concrete driver-specific settings (servo/DC)
// live in their respective config headers to avoid coupling.

// Select which motor driver to install at startup.
// Options:
//   0  -> legacy servo adapter    (installDefaultServoAdapter())
//   1  -> DC mirror driver        (installDefaultDCMirrorDriver())
//   2  -> MKS Servo serial driver (installDefaultMksServoDriver())
// Default: 1
#ifndef MOTOR_USE_DC_DRIVER
#define MOTOR_USE_DC_DRIVER 0
#endif

#ifndef MOTOR_USE_MKS_SERVO
#define MOTOR_USE_MKS_SERVO 1
#endif

