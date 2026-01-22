// mks_servo_constants.h - Technical constants for MKS SERVO drivers
#pragma once

#include <stdint.h>

/**
 * @brief Technical parameters for the MKS RS485 protocol and pulse generation.
 * These are "magic values" that typically don't change between robots 
 * but affect performance and stability.
 */

// Spec limit for MKS SERVO42D/57D step input. 
// Do not exceed 160kHz to avoid lost steps or driver stall.
#ifndef MKS_SERVO_STEP_MAX_HZ
#define MKS_SERVO_STEP_MAX_HZ 160000
#endif

// Timeout for standard RS485 data frames (us). 
// 2ms is appropriate for a 1000Hz control loop delay budget.
#ifndef MKS_SERVO_TIMEOUT_DATA_US
#define MKS_SERVO_TIMEOUT_DATA_US 2000
#endif

// Timeout for configuration or "heavy" operations (us).
#ifndef MKS_SERVO_TIMEOUT_HEAVY_US
#define MKS_SERVO_TIMEOUT_HEAVY_US 50000
#endif

// Throttle frequency updates: only update if change is > X Hz.
// 5Hz is a good balance between precision and bus/CPU noise.
#ifndef MKS_SERVO_STEP_DEADBAND_HZ
#define MKS_SERVO_STEP_DEADBAND_HZ 5
#endif

// Smoothing factor for speed estimator (0.0=none, 1.0=frozen).
#ifndef MKS_SERVO_SPEED_ALPHA
#define MKS_SERVO_SPEED_ALPHA 0.3f
#endif

// Minimum time between two frames on the same serial bus (us).
#ifndef MKS_SERVO_MIN_INTERFRAME_US
#define MKS_SERVO_MIN_INTERFRAME_US 500
#endif

// Diagnostic log interval (ms).
#ifndef MKS_SERVO_DIAG_LOG_ms
#define MKS_SERVO_DIAG_LOG_ms 2000
#endif

// Pulse generation bit depth (if using LEDC fallback)
#ifndef MKS_SERVO_LEDC_RES
#define MKS_SERVO_LEDC_RES 8
#endif

// Pulse generation duty (usually 50% for Step pulses)
#ifndef MKS_SERVO_LEDC_DUTY
#define MKS_SERVO_LEDC_DUTY 128 // 50% of 2^8
#endif

// Duration of pulsing pause around RS485 requests (us). 
// Set to 0 if the motor firmware can handle simultaneous Step/Dir and RS485.
#ifndef MKS_SERVO_TELEMETRY_QUIET_WINDOW_US
#define MKS_SERVO_TELEMETRY_QUIET_WINDOW_US 0
#endif

// Enable non-blocking ACK handling by default (1 = on, 0 = off).
#ifndef MKS_SERVO_DEFAULT_WAIT_FOR_ACK
#define MKS_SERVO_DEFAULT_WAIT_FOR_ACK 1
#endif

// Queue depth for function commands (mode, current, hold, etc.).
#ifndef MKS_SERVO_FUNCTION_QUEUE_LEN
#define MKS_SERVO_FUNCTION_QUEUE_LEN 8
#endif
