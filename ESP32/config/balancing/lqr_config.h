// lqr_config.h
// Configuration for the Cascaded LQR control strategy.
#pragma once

// --- Adaptive Pitch Trim ---
// Automatically adjusts the balance center to compensate for battery weight or drift.
#ifndef BALANCER_ENABLE_ADAPTIVE_TRIM
#define BALANCER_ENABLE_ADAPTIVE_TRIM 1
#endif

#ifndef BALANCER_ADAPTIVE_TRIM_ALPHA
#define BALANCER_ADAPTIVE_TRIM_ALPHA 0.001f  // Faster adaptation (10x from previous)
#endif

// --- Cascaded LQR Gains ---
#ifndef BALANCER_DEFAULT_K_PITCH
#define BALANCER_DEFAULT_K_PITCH 0.04f      // Kp: Pitch error (rad) -> Command
#endif

#ifndef BALANCER_DEFAULT_K_GYRO
#define BALANCER_DEFAULT_K_GYRO 0.005f      // Kg: Pitch rate (rad/s) -> Command (Damping)
#endif

#ifndef BALANCER_DEFAULT_K_DIST
#define BALANCER_DEFAULT_K_DIST 0.00002f   // Doubled to anchor more firmly
#endif

#ifndef BALANCER_DEFAULT_K_SPEED
#define BALANCER_DEFAULT_K_SPEED 0.0002f    // Doubled for better damping
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

// Low-pass filter alpha for pitch rate (1000Hz)
// 0.2 means 20% new sample, 80% old. Approx 30Hz cutoff.
#ifndef BALANCER_PITCH_RATE_LPF_ALPHA
#define BALANCER_PITCH_RATE_LPF_ALPHA 0.2f
#endif

// Yaw rate noise gate (rad/s)
#ifndef BALANCER_YAW_DEADBAND
#define BALANCER_YAW_DEADBAND 0.01f
#endif

// --- Control Saturation Limits ---

// Max command contribution from position error (Kd * dist)
#ifndef BALANCER_LQR_SAT_DIST
#define BALANCER_LQR_SAT_DIST 0.4f
#endif

// Max command contribution from speed error (Ks * vel)
#ifndef BALANCER_LQR_SAT_SPEED
#define BALANCER_LQR_SAT_SPEED 0.4f
#endif
