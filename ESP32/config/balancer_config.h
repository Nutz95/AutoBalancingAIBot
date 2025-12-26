// balancer_config.h
// Default configuration for the balancer PID controller.
#pragma once

// Default PID gains (units: pitch in radians)
// NOTE: These defaults are tuned for DIRECT VELOCITY control mode.
// Adjusted for CG @ 6.5cm above wheel axis, mass 1.3kg, wheel diameter 67mm
// Kp=2.1 means 1 degree error (0.017 rad) -> 0.036 command (3.6% speed)
// Saturation (100% speed) occurs at ~27.4 degrees error.
#ifndef BALANCER_DEFAULT_KP
#define BALANCER_DEFAULT_KP 9.8f
#endif
// Ki=0 by default to avoid integral windup causing direction reversal
// Add small Ki (0.01-0.05) only after Kp/Kd are tuned
#ifndef BALANCER_DEFAULT_KI
#define BALANCER_DEFAULT_KI 3.2f
#endif

#ifndef BALANCER_DEFAULT_KD
#define BALANCER_DEFAULT_KD 0.43f
#endif

// Integrator anti-windup clamp (absolute limit applied to integrator state)
#ifndef BALANCER_INTEGRATOR_LIMIT
#define BALANCER_INTEGRATOR_LIMIT 0.5f
#endif

// Whether to enable the balancer by default on startup (false recommended)
#ifndef BALANCER_ENABLE_BY_DEFAULT
#define BALANCER_ENABLE_BY_DEFAULT 0
#endif

// Minimum absolute normalized motor output to overcome motor deadband/holding
// Value from motor characterization (measured deadzone ~0.11-0.15, conservative 0.20)
// This is the physical threshold below which motors don't move.
#ifndef BALANCER_MOTOR_MIN_OUTPUT
#define BALANCER_MOTOR_MIN_OUTPUT 0.20f
#endif

// Command slew limit (normalized units per second).
// DISABLED (set very high) - balancer needs instant response for corrections.
// Slew rate limits prevent fast recovery from large pitch errors.
// If oscillations occur, fix PID gains instead of adding slew rate.
#ifndef BALANCER_CMD_SLEW_LIMIT
#define BALANCER_CMD_SLEW_LIMIT 10000.0f
#endif

// Angle limit (degrees) for auto-enabling motors when starting balancer.
// If the absolute fused pitch is greater than this value, motors will not
// be auto-enabled to avoid trying to balance a fallen robot.
#ifndef BALANCER_AUTO_ENABLE_ANGLE_DEG
#define BALANCER_AUTO_ENABLE_ANGLE_DEG 2.0f
#endif

// Smaller threshold (degrees) indicating the robot must be close to vertical
// before permitting motors to enable after a start request.
#ifndef BALANCER_START_STABLE_ANGLE_DEG
#define BALANCER_START_STABLE_ANGLE_DEG 2.0f
#endif

// Maximum absolute pitch rate (deg/s) allowed for the initial enable window.
// Lowered from 8.0 to 3.0 to ensure robot is truly stable (not falling) when
// motors enable. With 8.0, robot could be falling at -8.14 deg/s and motors
// would still enable, causing immediate instability and violent oscillations.
#ifndef BALANCER_START_STABLE_PITCH_RATE_DEG_S
#define BALANCER_START_STABLE_PITCH_RATE_DEG_S 3.0f
#endif

// Maximum absolute trim applied at start (degrees). The controller will capture
// the current fused pitch at BALANCE START, clamp it to this value, and subtract
// it from subsequent control to compensate for minor floor tilt or sensor bias.
#ifndef BALANCER_TRIM_MAX_DEG
#define BALANCER_TRIM_MAX_DEG 8.0f
#endif

// If the robot exceeds this absolute pitch (degrees), automatically stop the
// balancer to avoid fighting on the ground after a fall.
#ifndef BALANCER_FALL_STOP_ANGLE_DEG
#define BALANCER_FALL_STOP_ANGLE_DEG 30.0f
#endif

// Optional pitch-rate guard (deg/s). Set to 0 to disable rate-based stop.
#ifndef BALANCER_FALL_STOP_RATE_DEG_S
#define BALANCER_FALL_STOP_RATE_DEG_S 0.0f
#endif

// Debug logging interval (milliseconds)
// Controls how often BALANCER_DBG logs are emitted to reduce console spam
// Set to 0 to disable throttling, 1 for max rate (1000 Hz), 500 for 2 Hz
// For motor characterization scripts, use 1-2ms (500-1000 Hz)
#ifndef BALANCER_DEBUG_LOG_INTERVAL_MS
#define BALANCER_DEBUG_LOG_INTERVAL_MS 500
#endif

// Drive interface configuration
// How quickly the normalized forward command (v) may change (units/sec).
// Higher = more responsive. Safe default = 2.0f (full step ~0.5s).
#ifndef DRIVE_V_SLEW
#define DRIVE_V_SLEW 1.5f
#endif

// Maximum pitch (degrees) the drive command will request from the balancer.
// This limits how far the robot will lean to accelerate/decelerate. Default 6°.
#ifndef DRIVE_MAX_PITCH_DEG
#define DRIVE_MAX_PITCH_DEG 3.0f
#endif

// Motor command inversion for balancer (set to 1 if motor is mounted in mirror)
// LEFT motor: 0 = normal, 1 = invert command sign
// RIGHT motor: 0 = normal, 1 = invert command sign
#ifndef BALANCER_LEFT_MOTOR_INVERT
#define BALANCER_LEFT_MOTOR_INVERT 0
#endif
#ifndef BALANCER_RIGHT_MOTOR_INVERT
#define BALANCER_RIGHT_MOTOR_INVERT 1
#endif

// Motor command gain scaling factors (to compensate for asymmetric motor response)
// Initialized from motor characterization ratios if available, else 1.0
// Use 1.0 for both if motors are matched. Adjust to balance robot behavior:
// - If robot drifts left: increase LEFT_GAIN or decrease RIGHT_GAIN
// - If robot drifts right: increase RIGHT_GAIN or decrease LEFT_GAIN
// Example: LEFT=1.0, RIGHT=0.95 means right motor gets 95% of commanded value
#ifndef BALANCER_LEFT_MOTOR_GAIN
#if defined(MOTOR_CHAR_LEFT_GAIN_COUNTS_PER_CMD) && defined(MOTOR_CHAR_RIGHT_GAIN_COUNTS_PER_CMD)
// Auto-scale based on measured motor gains (normalize to average)
#define BALANCER_LEFT_MOTOR_GAIN \
  (2.0f * MOTOR_CHAR_LEFT_GAIN_COUNTS_PER_CMD / \
   (MOTOR_CHAR_LEFT_GAIN_COUNTS_PER_CMD + MOTOR_CHAR_RIGHT_GAIN_COUNTS_PER_CMD))
#else
#define BALANCER_LEFT_MOTOR_GAIN 1.0f
#endif
#endif
#ifndef BALANCER_RIGHT_MOTOR_GAIN
#if defined(MOTOR_CHAR_LEFT_GAIN_COUNTS_PER_CMD) && defined(MOTOR_CHAR_RIGHT_GAIN_COUNTS_PER_CMD)
// Auto-scale based on measured motor gains (normalize to average)
#define BALANCER_RIGHT_MOTOR_GAIN \
  (2.0f * MOTOR_CHAR_RIGHT_GAIN_COUNTS_PER_CMD / \
   (MOTOR_CHAR_LEFT_GAIN_COUNTS_PER_CMD + MOTOR_CHAR_RIGHT_GAIN_COUNTS_PER_CMD))
#else
#define BALANCER_RIGHT_MOTOR_GAIN 1.18f
#endif
#endif
