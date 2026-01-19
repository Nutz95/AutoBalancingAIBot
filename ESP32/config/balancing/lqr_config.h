// lqr_config.h
// Configuration for the Cascaded LQR control strategy.
#pragma once

// --- Adaptive Pitch Trim ---
// Automatically adjusts the balance center to compensate for battery weight or drift.
#ifndef BALANCER_ENABLE_ADAPTIVE_TRIM
#define BALANCER_ENABLE_ADAPTIVE_TRIM 1
#endif

#ifndef BALANCER_ADAPTIVE_TRIM_ALPHA
#define BALANCER_ADAPTIVE_TRIM_ALPHA 0.0008f  // Speed up convergence 4x (from 0.0002)
#endif

// Limits for adaptive trim activation
#ifndef BALANCER_ADAPTIVE_TRIM_MAX_PITCH_DEG
#define BALANCER_ADAPTIVE_TRIM_MAX_PITCH_DEG 7.0f   // Only learn COG when nearly vertical
#endif

#ifndef BALANCER_ADAPTIVE_TRIM_MAX_DIST_TICKS
#define BALANCER_ADAPTIVE_TRIM_MAX_DIST_TICKS 2000.0f // Back to reasonable range
#endif

#ifndef BALANCER_ADAPTIVE_TRIM_LIMIT_RAD
#define BALANCER_ADAPTIVE_TRIM_LIMIT_RAD 0.4f // approx 23 degrees max (handling the 13.7 bias)
#endif

// --- Cascaded LQR Gains ---
#ifndef BALANCER_DEFAULT_K_PITCH
#define BALANCER_DEFAULT_K_PITCH 0.045f     // Firm control
#endif

#ifndef BALANCER_DEFAULT_K_GYRO
#define BALANCER_DEFAULT_K_GYRO 0.012f      // Increased damping (was 0.008)
#endif

#ifndef BALANCER_DEFAULT_K_DIST
#define BALANCER_DEFAULT_K_DIST 0.000005f   // Keep it very soft
#endif

#ifndef BALANCER_DEFAULT_K_SPEED
#define BALANCER_DEFAULT_K_SPEED 0.0f       // Keep speed damping at 0 for cleaner debug
#endif

// --- Yaw / Heading Control ---
#ifndef BALANCER_DEFAULT_K_YAW
#define BALANCER_DEFAULT_K_YAW 0.4f         // Ky: Yaw error -> Steer output (Heading Hold)
#endif

#ifndef BALANCER_DEFAULT_K_YAW_RATE
#define BALANCER_DEFAULT_K_YAW_RATE 0.1f    // Kyr: Yaw rate -> Steer output (Yaw damping)
#endif

// --- Mechanical & Filter Parameters ---

// Wheel Diameter in meters (67mm = 0.067m)
#ifndef BALANCER_WHEEL_DIAMETER_M
#define BALANCER_WHEEL_DIAMETER_M 0.067f
#endif

// Encoder resolution (ticks per revolution)
#ifndef BALANCER_TICKS_PER_REV
#define BALANCER_TICKS_PER_REV 51200.0f
#endif

// Logic downscaling factor for ultra-high-res encoders (reduces derivative noise & gain magnitude)
#ifndef BALANCER_ENCODER_DOWNSCALE
#define BALANCER_ENCODER_DOWNSCALE 10.0f
#endif

// Scaling for joystick velocity input (maps normalized 0..1 to virtual ticks/s)
#ifndef BALANCER_VELOCITY_TARGET_SCALE
#define BALANCER_VELOCITY_TARGET_SCALE 2000.0f
#endif

// Low-pass filter alpha for derivatives (500Hz)
#ifndef BALANCER_PITCH_RATE_LPF_ALPHA
#define BALANCER_PITCH_RATE_LPF_ALPHA 0.1f  // Strong filters to kill the 15Hz vibration
#endif

#ifndef BALANCER_SPEED_LPF_ALPHA
#define BALANCER_SPEED_LPF_ALPHA 0.02f      // Very clean speed signal
#endif

// Yaw rate noise gate (rad/s)
#ifndef BALANCER_YAW_DEADBAND
#define BALANCER_YAW_DEADBAND 0.05f  // Increased (0.01 -> 0.05) to stop 3D twitching
#endif

// --- Control Saturation Limits ---

// Max command contribution from position error (Kd * dist)
#ifndef BALANCER_LQR_SAT_DIST
#define BALANCER_LQR_SAT_DIST 0.2f  // Lowered to avoid stealing authority from angle correction
#endif

// Max command contribution from speed error (Ks * vel)
#ifndef BALANCER_LQR_SAT_SPEED
#define BALANCER_LQR_SAT_SPEED 0.4f
#endif
