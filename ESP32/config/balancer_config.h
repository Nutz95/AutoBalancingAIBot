// balancer_config.h
// Default configuration for the balancer PID controller.
#pragma once

#include "drive_config.h"

// Minimum command applied when outside deadband (feedforward to overcome static friction)
// For NEMA FOC, try 0 first as the controller is very sensitive.
#ifndef BALANCER_MIN_CMD
#define BALANCER_MIN_CMD 0.0f
#endif

// Default PID gains (units: pitch in degrees)
// Adjusted for NEMA FOC: lower gains to handle high motor bandwidth
#ifndef BALANCER_DEFAULT_KP
#define BALANCER_DEFAULT_KP 0.04f
#endif

#ifndef BALANCER_DEFAULT_KI
#define BALANCER_DEFAULT_KI 0.08f
#endif

#ifndef BALANCER_DEFAULT_KD
#define BALANCER_DEFAULT_KD 0.006f
#endif

// Integrator anti-windup clamp (absolute limit applied to integrator state)
#ifndef BALANCER_INTEGRATOR_LIMIT
#define BALANCER_INTEGRATOR_LIMIT 1.0f
#endif

// Coeff d'atténuation de l'intégrateur (Leaky Integrator)
// 1.0 = pas d'atténuation. 0.9998 = ~8% de perte par seconde à 400Hz.
// Aide à prévenir la dérive à long terme et les blocages de l'intégrateur.
#ifndef BALANCER_INTEGRATOR_LEAK_COEFF
#define BALANCER_INTEGRATOR_LEAK_COEFF 0.9998f
#endif

// Whether to enable the balancer by default on startup (false recommended)
#ifndef BALANCER_ENABLE_BY_DEFAULT
#define BALANCER_ENABLE_BY_DEFAULT 0
#endif

// Minimum absolute normalized motor output to overcome motor deadband/holding
// For FOC, start at 0.0 to avoid oscillations around center.
#ifndef BALANCER_MOTOR_MIN_OUTPUT
#define BALANCER_MOTOR_MIN_OUTPUT 0.0f
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
// 20ms (50Hz) is recommended to avoid Serial/WiFi congestion.
#ifndef BALANCER_DEBUG_LOG_INTERVAL_MS
#define BALANCER_DEBUG_LOG_INTERVAL_MS 20
#endif

// Performance & Telemetry Optimization
// If 1, the balancer loop will regularly read wheel encoders during balancing.
// WARNING: This adds RS485 bus latency (~600us per motor at 256000 baud).
// Recommended only when investigating "sliding" or using cascaded position loop.
#ifndef BALANCER_ENABLE_ENCODER_UPDATES
#define BALANCER_ENABLE_ENCODER_UPDATES 1
#endif

// Interval for encoder updates (ms). Throttles bus traffic.
// Only used if BALANCER_ENABLE_ENCODER_UPDATES is 1.
// Increased to 100ms to reduce loop jitter and improve pitch stability.
#ifndef BALANCER_ENCODER_UPDATE_MS
#define BALANCER_ENCODER_UPDATE_MS 100
#endif

// Smoothing factor for pitch rate (D-term input). 
// 1.0 = no filtering, lower = more smoothing.
// Recommended: 0.2 to 0.5 to reduce motor jitter from raw gyro noise.
#ifndef BALANCER_PITCH_RATE_ALPHA
#define BALANCER_PITCH_RATE_ALPHA 0.4f
#endif

// (Duplicate flag removed)

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
#define BALANCER_RIGHT_MOTOR_GAIN 1.0f
#endif
#endif
