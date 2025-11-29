// balancer_config.h
// Default configuration for the balancer PID controller.
#pragma once

// Default PID gains (units: pitch in radians)
#ifndef BALANCER_DEFAULT_KP
#define BALANCER_DEFAULT_KP 0.5f
#endif

#ifndef BALANCER_DEFAULT_KI
#define BALANCER_DEFAULT_KI 0.0f
#endif

#ifndef BALANCER_DEFAULT_KD
#define BALANCER_DEFAULT_KD 0.05f
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
#ifndef BALANCER_MOTOR_MIN_OUTPUT
#define BALANCER_MOTOR_MIN_OUTPUT 0.05f
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
#define BALANCER_AUTO_ENABLE_ANGLE_DEG 20.0f
#endif
