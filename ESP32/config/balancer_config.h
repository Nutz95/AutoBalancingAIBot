// balancer_config.h
// Default configuration for the balancer PID controller.
#pragma once

#include "drive_config.h"

// Minimum command applied when outside deadband (feedforward to overcome static friction)
// For NEMA FOC, try 0 first as the controller is very sensitive.
#ifndef BALANCER_MIN_CMD
#define BALANCER_MIN_CMD 0.0f
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
// Increased from 2.0 to 10.0 to be more permissive during tuning.
#ifndef BALANCER_AUTO_ENABLE_ANGLE_DEG
#define BALANCER_AUTO_ENABLE_ANGLE_DEG 10.0f
#endif

// Smaller threshold (degrees) indicating the robot must be close to vertical
// before permitting motors to enable after a start request.
#ifndef BALANCER_START_STABLE_ANGLE_DEG
#define BALANCER_START_STABLE_ANGLE_DEG 10.0f
#endif

// Maximum absolute pitch rate (deg/s) allowed for the initial enable window.
// Lowered from 8.0 to 3.0 to ensure robot is truly stable (not falling) when
// motors enable. With 8.0, robot could be falling at -8.14 deg/s and motors
// would still enable, causing immediate instability and violent oscillations.
#ifndef BALANCER_START_STABLE_PITCH_RATE_DEG_S
#define BALANCER_START_STABLE_PITCH_RATE_DEG_S 3.0f
#endif

// Logic divider for control loop frequency (relative to IMU sampling rate)
// If IMU is at 1000Hz, a divider of 1 runs the balancer at 1000Hz.
#ifndef BALANCER_CONTROL_DIVIDER
#define BALANCER_CONTROL_DIVIDER 1
#endif

// Maximum absolute trim applied at start (degrees). The controller will capture
// the current fused pitch at BALANCE START, clamp it to this value, and subtract
// it from subsequent control to compensate for minor floor tilt or sensor bias.
#ifndef BALANCER_TRIM_MAX_DEG
#define BALANCER_TRIM_MAX_DEG 20.0f
#endif

// If the robot exceeds this absolute pitch (degrees), automatically stop the
// balancer to avoid fighting on the ground after a fall.
// Increased from 30.0 to 45.0 for better tuning headroom.
#ifndef BALANCER_FALL_STOP_ANGLE_DEG
#define BALANCER_FALL_STOP_ANGLE_DEG 45.0f
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
// Reduced to 25ms (40Hz) to allow for a future velocity control loop.
#ifndef BALANCER_ENCODER_UPDATE_MS
#define BALANCER_ENCODER_UPDATE_MS 25
#endif

// Encoder Outlier Rejection
// Threshold for detecting impossible jumps in encoder values (telemetry glitches)
#ifndef BALANCER_ENCODER_GLITCH_THRESHOLD
#define BALANCER_ENCODER_GLITCH_THRESHOLD 1000
#endif
#ifndef BALANCER_VELOCITY_GLITCH_THRESHOLD
#define BALANCER_VELOCITY_GLITCH_THRESHOLD 50000
#endif

// Smoothing factor for pitch rate (D-term input). 
// 1.0 = no filtering, lower = more smoothing.
// If robot vibrates/buzzes, lower this value (e.g. 0.05 to 0.1).
#ifndef BALANCER_PITCH_RATE_ALPHA
#define BALANCER_PITCH_RATE_ALPHA 0.08f // (au lieu de 0.15f)
#endif

// --- Controller Strategy ---
// 0: LEGACY_PID (Basic pitch PID)
// 1: CASCADED_LQR (Navbot-style: Angle + Gyro + Distance + Speed)
#ifndef BALANCER_CONTROLLER_MODE
#define BALANCER_CONTROLLER_MODE 1
#endif

#include "balancing/pid_config.h"
#include "balancing/lqr_config.h"

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

// --- High-Speed Binary Telemetry Optimization ---
// Binary telemetry is sent via UDP at a fraction of the IMU loop rate.
// 1000Hz Loop / 5 = 200Hz Telemetry. This keeps Core 0 load manageable.
#ifndef TELEMETRY_BINARY_DIVIDER
#define TELEMETRY_BINARY_DIVIDER 5
#endif

// Enable/disable UDP binary telemetry at compile time (A/B test CPU load).
#ifndef TELEMETRY_UDP_ENABLED
#define TELEMETRY_UDP_ENABLED 1
#endif

// Throttles text diagnostics logging to serial/console (ms).
// Set to a high value (e.g. 1000) when high-speed binary is active.
#ifndef TELEMETRY_TEXT_THROTTLE_MS
#define TELEMETRY_TEXT_THROTTLE_MS 1000
#endif

// Enable/disable text diagnostics logging at compile time (A/B test CPU load).
#ifndef TELEMETRY_TEXT_ENABLED
#define TELEMETRY_TEXT_ENABLED 1
#endif
