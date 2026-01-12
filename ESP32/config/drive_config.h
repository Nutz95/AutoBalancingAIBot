// drive_config.h
// Configuration for joystick-driven motion and fluidity enhancements.
#pragma once

// --- Joystick Input ---
// Joystick deadzone (normalized 0..1). Commands below this absolute value
// will be treated as zero to account for joystick centering drift.
#ifndef DRIVE_JOYSTICK_DEADZONE
#define DRIVE_JOYSTICK_DEADZONE 0.05f
#endif

// How quickly the normalized forward command (v) may change (units/sec).
// Higher = more responsive. Safe default = 2.0f (full step ~0.5s).
#ifndef DRIVE_V_SLEW
#define DRIVE_V_SLEW 3.0f
#endif

// Maximum pitch (degrees) the drive command will request from the balancer.
// This limits how far the robot will lean to accelerate/decelerate.
#ifndef DRIVE_MAX_PITCH_DEG
#define DRIVE_MAX_PITCH_DEG 5.0f
#endif


// --- Fluidity Enhancements ---

// Direction change "Kick": help overcome static friction when reversing.
// Noise floor for detecting a significant command.
#ifndef DRIVE_KICK_NOISE_FLOOR
#define DRIVE_KICK_NOISE_FLOOR 0.02f
#endif
// Multiplier applied to the command during a direction change.
#ifndef DRIVE_KICK_BOOST
#define DRIVE_KICK_BOOST 1.0f
#endif

// Deceleration Boost: help the robot return to vertical when joystick is released.
// Noise floor for detecting that the robot is still moving.
#ifndef DRIVE_BRAKE_NOISE_FLOOR
#define DRIVE_BRAKE_NOISE_FLOOR 0.01f
#endif
// Multiplier applied to the command during the braking phase.
#ifndef DRIVE_BRAKE_BOOST
#define DRIVE_BRAKE_BOOST 1.0f
#endif


// --- Safety & Stabilization ---

// Angle (degrees) at which the drive setpoint starts being attenuated
// to prioritize stabilization over user command.
#ifndef DRIVE_SAFETY_ATTENUATION_START_DEG
#define DRIVE_SAFETY_ATTENUATION_START_DEG 5.0f
#endif

// Angle (degrees) at which the drive setpoint is fully ignored (0.0).
#ifndef DRIVE_SAFETY_ATTENUATION_END_DEG
#define DRIVE_SAFETY_ATTENUATION_END_DEG 15.0f
#endif

// Angle (degrees) at which the PID integrator is reset to avoid violent recovery.
#ifndef DRIVE_SAFETY_INTEGRATOR_RESET_DEG
#define DRIVE_SAFETY_INTEGRATOR_RESET_DEG 15.0f
#endif
