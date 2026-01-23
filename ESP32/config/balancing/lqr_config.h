// lqr_config.h
// Configuration for the Cascaded LQR control strategy.
#pragma once

// --- Adaptive Pitch Trim ---
// Automatically adjusts the balance center to compensate for battery weight or drift.
#ifndef BALANCER_ENABLE_ADAPTIVE_TRIM
#define BALANCER_ENABLE_ADAPTIVE_TRIM 1
#endif

#ifndef BALANCER_ADAPTIVE_TRIM_ALPHA
#define BALANCER_ADAPTIVE_TRIM_ALPHA 0.000001f // v73: 100x slower to be stable
#endif

// Limits for adaptive trim activation
#ifndef BALANCER_ADAPTIVE_TRIM_MAX_PITCH_DEG
#define BALANCER_ADAPTIVE_TRIM_MAX_PITCH_DEG 15.0f   // v72: Reasonable window
#endif

#ifndef BALANCER_ADAPTIVE_TRIM_MAX_DIST_TICKS
#define BALANCER_ADAPTIVE_TRIM_MAX_DIST_TICKS 2000000.0f // v72: Virtually unlimited to allow finding center
#endif

#ifndef BALANCER_ADAPTIVE_TRIM_LIMIT_RAD
#define BALANCER_ADAPTIVE_TRIM_LIMIT_RAD 0.4f // approx 23 degrees max
#endif

#ifndef BALANCER_ADAPTIVE_TRIM_DEADBAND_TICKS
#define BALANCER_ADAPTIVE_TRIM_DEADBAND_TICKS 30.0f 
#endif

// --- Cascaded LQR Gains ---
#ifndef BALANCER_DEFAULT_K_PITCH
#define BALANCER_DEFAULT_K_PITCH 0.050f     // v76: Keep authority
#endif

#ifndef BALANCER_DEFAULT_K_GYRO
#define BALANCER_DEFAULT_K_GYRO 0.025f      // v76: reduced damping to favor recovery
#endif

#ifndef BALANCER_DEFAULT_K_DIST
#define BALANCER_DEFAULT_K_DIST 0.000100f    // v76: softer anchor
#endif

#ifndef BALANCER_DEFAULT_K_SPEED
#define BALANCER_DEFAULT_K_SPEED 0.000010f  // v76: much less speed damping (was fighting recovery)
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
#define BALANCER_PITCH_RATE_LPF_ALPHA 0.20f  // v76: more filtering for cleaner response
#endif

// Optional: Use a cutoff frequency (Hz) for pitch-rate low-pass instead of a fixed alpha.
// Set to 0 to keep using BALANCER_PITCH_RATE_LPF_ALPHA.
#ifndef BALANCER_LQR_PITCH_RATE_LPF_HZ
#define BALANCER_LQR_PITCH_RATE_LPF_HZ 0.0f
#endif

// Optional: Low-pass filter the final LQR command (Hz). Set to 0 to disable.
// This can reduce high-frequency oscillations but adds phase lag.
#ifndef BALANCER_LQR_CMD_LPF_HZ
#define BALANCER_LQR_CMD_LPF_HZ 0.0f
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
#define BALANCER_LQR_SAT_DIST 0.5f  // v73: Allow more authority to pull back (was 0.2)
#endif

// Max command contribution from speed error (Ks * vel)
#ifndef BALANCER_LQR_SAT_SPEED
#define BALANCER_LQR_SAT_SPEED 0.4f
#endif
