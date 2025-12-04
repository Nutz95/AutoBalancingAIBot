// balancer_config.h
// Default configuration for the balancer PID controller.
#pragma once

// Default PID gains (units: pitch in radians)
// NOTE: These defaults are tuned for DIRECT VELOCITY control mode.
// Kp=8.0 means 1 degree error (0.017 rad) -> 0.13 command (13% speed)
// Saturation (100% speed) occurs at ~7.7 degrees error.
#ifndef BALANCER_DEFAULT_KP
#define BALANCER_DEFAULT_KP 40.0f
#endif

#ifndef BALANCER_DEFAULT_KI
#define BALANCER_DEFAULT_KI 0.02f
#endif

#ifndef BALANCER_DEFAULT_KD
#define BALANCER_DEFAULT_KD 2.0f
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
