// balancer_config.h
// Default configuration for the balancer PID controller.
#pragma once

// Default PID gains (units: pitch in radians)
// NOTE: These defaults are tuned for DIRECT VELOCITY control mode.
// Kp=8.0 means 1 degree error (0.017 rad) -> 0.13 command (13% speed)
// Saturation (100% speed) occurs at ~7.7 degrees error.
#ifndef BALANCER_DEFAULT_KP
#define BALANCER_DEFAULT_KP 25.5f
#endif

// Ki=0 by default to avoid integral windup causing direction reversal
// Add small Ki (0.01-0.05) only after Kp/Kd are tuned
#ifndef BALANCER_DEFAULT_KI
#define BALANCER_DEFAULT_KI 1.0f
#endif

#ifndef BALANCER_DEFAULT_KD
#define BALANCER_DEFAULT_KD 0.9f
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
// NOTE: In POSITION mode with servo asservissement, deadband can be reduced
// since the servo actively holds position even with small commands.
#ifndef BALANCER_MOTOR_MIN_OUTPUT
#define BALANCER_MOTOR_MIN_OUTPUT 0.02f
#endif

// Command slew limit (normalized units per second). Prevents instant full-power
// steps from small transients. Set to 10.0f for faster response (command can change by 10.0 per second).
#ifndef BALANCER_CMD_SLEW_LIMIT
#define BALANCER_CMD_SLEW_LIMIT 10.0f
#endif

// Angle limit (degrees) for auto-enabling motors when starting balancer.
// If the absolute fused pitch is greater than this value, motors will not
// be auto-enabled to avoid trying to balance a fallen robot.
#ifndef BALANCER_AUTO_ENABLE_ANGLE_DEG
#define BALANCER_AUTO_ENABLE_ANGLE_DEG 30.0f
#endif

// Smaller threshold (degrees) indicating the robot must be close to vertical
// before permitting motors to enable after a start request.
#ifndef BALANCER_START_STABLE_ANGLE_DEG
#define BALANCER_START_STABLE_ANGLE_DEG 3.0f
#endif

// Maximum absolute pitch rate (deg/s) allowed for the initial enable window.
#ifndef BALANCER_START_STABLE_PITCH_RATE_DEG_S
#define BALANCER_START_STABLE_PITCH_RATE_DEG_S 8.0f
#endif

// If the robot exceeds this absolute pitch (degrees), automatically stop the
// balancer to avoid fighting on the ground after a fall.
#ifndef BALANCER_FALL_STOP_ANGLE_DEG
#define BALANCER_FALL_STOP_ANGLE_DEG 45.0f
#endif

// Optional pitch-rate guard (deg/s). Set to 0 to disable rate-based stop.
#ifndef BALANCER_FALL_STOP_RATE_DEG_S
#define BALANCER_FALL_STOP_RATE_DEG_S 0.0f
#endif

// Drive interface configuration
// How quickly the normalized forward command (v) may change (units/sec).
// Higher = more responsive. Safe default = 2.0f (full step ~0.5s).
#ifndef DRIVE_V_SLEW
#define DRIVE_V_SLEW 3.2f
#endif

// Maximum pitch (degrees) the drive command will request from the balancer.
// This limits how far the robot will lean to accelerate/decelerate. Default 6°.
#ifndef DRIVE_MAX_PITCH_DEG
#define DRIVE_MAX_PITCH_DEG 3.0f
#endif
