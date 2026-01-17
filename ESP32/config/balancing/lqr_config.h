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
#define BALANCER_DEFAULT_K_DIST 0.00001f   // Further increased to stop movement
#endif

#ifndef BALANCER_DEFAULT_K_SPEED
#define BALANCER_DEFAULT_K_SPEED 0.0001f    // Further increased to add damping
#endif

// --- Yaw / Heading Control ---
#ifndef BALANCER_DEFAULT_K_YAW
#define BALANCER_DEFAULT_K_YAW 0.4f         // Ky: Yaw error -> Steer output (Heading Hold)
#endif

#ifndef BALANCER_DEFAULT_K_YAW_RATE
#define BALANCER_DEFAULT_K_YAW_RATE 0.1f    // Kyr: Yaw rate -> Steer output (Yaw damping)
#endif
