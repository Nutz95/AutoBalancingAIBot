// mks_servo_config.h - Configuration for MKS SERVO42D/57D serial drivers
#pragma once

#include <stdint.h>

namespace abbot {
namespace motor {

/**
 * @brief Operating modes for MKS SERVO42D drivers.
 * Used with function 0x82.
 */
enum class MksServoMode : uint8_t {
    CR_OPEN  = 0, // Current loop open-loop
    CR_CLOSE = 1, // Current loop closed-loop
    CR_vFOC  = 2, // Current loop vector FOC
    SR_OPEN  = 3, // Speed loop open-loop
    SR_CLOSE = 4, // Speed loop closed-loop
    SR_vFOC  = 5  // Speed loop vector FOC (Recommended for balancing)
};

/**
 * @brief Microstepping values. 
 * Used with function 0x84. 
 * Note: 0 is used for 256 microsteps.
 */
enum class MksServoMicrostep : uint8_t {
    STEP_1   = 1,
    STEP_2   = 2,
    STEP_4   = 4,
    STEP_8   = 8,
    STEP_16  = 16,
    STEP_32  = 32,
    STEP_64  = 64,
    STEP_128 = 128,
    STEP_256 = 0
};

/**
 * @brief Holding current percentage.
 * Used with function 0x9B.
 */
enum class MksServoHoldCurrent : uint8_t {
    PCT_10 = 0,
    PCT_20 = 1,
    PCT_30 = 2,
    PCT_40 = 3,
    PCT_50 = 4,
    PCT_60 = 5,
    PCT_70 = 6,
    PCT_80 = 7,
    PCT_90 = 8
};

} // namespace motor
} // namespace abbot

// =============================================================================
// HARDWARE CONNECTION
// =============================================================================

// Motor 1 (Left) - Serial Port 2
// Moved to GPIO 39/40 to avoid EMI from SPI bus (pins 4-7)
#ifndef MKS_SERVO_P1_TX_PIN
#define MKS_SERVO_P1_TX_PIN 39
#endif

#ifndef MKS_SERVO_P1_RX_PIN
#define MKS_SERVO_P1_RX_PIN 40
#endif

// Motor 2 (Right) - Serial Port 1
// Moved to GPIO 41/42 to avoid EMI from SPI bus (pins 4-7)
#ifndef MKS_SERVO_P2_TX_PIN
#define MKS_SERVO_P2_TX_PIN 41
#endif

#ifndef MKS_SERVO_P2_RX_PIN
#define MKS_SERVO_P2_RX_PIN 42
#endif

#ifndef MKS_SERVO_BAUD
#define MKS_SERVO_BAUD 256000
#endif

// NOTE: Both motors are on SEPARATE RS485 buses (Left=Serial2, Right=Serial1).
// Each bus has dedicated bandwidth. IDs can be different or identical - doesn't matter
// since buses are independent. DO NOT CHANGE these IDs without user authorization.
#ifndef LEFT_MOTOR_ID
#define LEFT_MOTOR_ID 0x01
#endif

#ifndef RIGHT_MOTOR_ID
#define RIGHT_MOTOR_ID 0x02
#endif

// =============================================================================
// MOTOR PARAMETERS
// =============================================================================

// =============================================================================
// MOTOR PARAMETERS (!!! DO NOT CHANGE INVERSION SETTINGS WITHOUT NOTIFYING USER !!!)
// These are calibrated for physical wiring.
// =============================================================================

// Default operating mode (SR_vFOC recommended)
#ifndef MKS_SERVO_DEFAULT_MODE
#define MKS_SERVO_DEFAULT_MODE abbot::motor::MksServoMode::SR_vFOC
#endif

// Microstepping (32 recommended for balancing smoothness)
#ifndef MKS_SERVO_DEFAULT_MSTEP
#define MKS_SERVO_DEFAULT_MSTEP abbot::motor::MksServoMicrostep::STEP_32
#endif

// Running current (mA) - Max 1500 for your setup
#ifndef MKS_SERVO_MA
#define MKS_SERVO_MA 1500
#endif

// Holding current percentage
#ifndef MKS_SERVO_HOLD_PCT
#define MKS_SERVO_HOLD_PCT abbot::motor::MksServoHoldCurrent::PCT_40
#endif

// Direction inversion (Physical motor orientation)
#ifndef LEFT_MOTOR_INVERT
#define LEFT_MOTOR_INVERT 1
#endif

#ifndef RIGHT_MOTOR_INVERT
#define RIGHT_MOTOR_INVERT 0
#endif

// =============================================================================
// PERFORMANCE & SAFETY
// =============================================================================

// Max speed in RPM (MKS protocol uses 12-bit value 0-3000)
// Increased to 1200 for better recovery response to disturbances
#ifndef VELOCITY_MAX_SPEED
#define VELOCITY_MAX_SPEED 1200
#endif

// Encoder update frequency (Hz). 
// High values (100-250) improve LQR/speed estimation but increase bus traffic.
#ifndef MKS_SERVO_ENCODER_UPDATE_HZ
#define MKS_SERVO_ENCODER_UPDATE_HZ 100
#endif

// Default acceleration for speed commands (0-255)
#ifndef MKS_SERVO_ACCEL
#define MKS_SERVO_ACCEL 255
#endif

// Timeout for logging diagnostic errors (ms) to avoid console spam
#ifndef MKS_SERVO_DIAG_LOG_ms
#define MKS_SERVO_DIAG_LOG_ms 2000
#endif

// Read encoder interval (ms) - Throttled to keep bus clear for commands
// Minimum time between encoder reads (milliseconds) to avoid bus saturation.
// Increased to 100ms to maximize bandwidth for balancing at 100Hz.
#ifndef MKS_SERVO_ENCODER_READ_MS
#define MKS_SERVO_ENCODER_READ_MS 100
#endif

// =============================================================================
// COMMUNICATION TIMING (Calculated based on MKS_SERVO_BAUD)
// =============================================================================
// Formula: (bits / baud) * 1,000,000 + margin
// For 256000 bps, 1 bit = 3.9us. 
// A 10-byte transaction (100 bits) takes ~390us raw wire time.
// Motor response delay (internal processing) is typically 200-500us.

// Critical command timeout (e.g. speed command)
// Reduced to 1500us to ensure we don't break the 500Hz (2ms) loop budget
// if a motor response is delayed.
#ifndef MKS_SERVO_TIMEOUT_CONTROL_US
#define MKS_SERVO_TIMEOUT_CONTROL_US 1500
#endif

// Telemetry/Data timeout (e.g. reading position)
// Position reads can take longer to process internally on MKS drivers
#ifndef MKS_SERVO_TIMEOUT_DATA_US
#define MKS_SERVO_TIMEOUT_DATA_US 10000  // Safe timeout for position reads
#endif

// Heavy request timeout (e.g. scanning or register dump)
#ifndef MKS_SERVO_TIMEOUT_HEAVY_US
#define MKS_SERVO_TIMEOUT_HEAVY_US 10000
#endif

// Bus quiet time between consecutive reads/writes to different IDs
// Increased to ensure no collisions when sending speed + encoder requests
#ifndef MKS_SERVO_BUS_QUIET_US
#define MKS_SERVO_BUS_QUIET_US 500
#endif

// Wait time after sending a telemetry request before trying to read the response.
// At 256kbaud, a 4-byte request takes ~160us. 500us ensures the frame is fully sent
// and the motor has time to process and begin its response.
#ifndef MKS_SERVO_TELEMETRY_WAIT_US
#define MKS_SERVO_TELEMETRY_WAIT_US 500
#endif

// Turnaround delay for RS485 auto-direction hardware (us)
#ifndef MKS_SERVO_RS485_TURNAROUND_US
#define MKS_SERVO_RS485_TURNAROUND_US 200
#endif

// How long to wait for bus availability (ms)
#ifndef MKS_SERVO_BUS_MUTEX_TIMEOUT_MS
#define MKS_SERVO_BUS_MUTEX_TIMEOUT_MS 20
#endif
