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

#ifndef MKS_SERVO_TX_PIN
#define MKS_SERVO_TX_PIN 17
#endif

#ifndef MKS_SERVO_RX_PIN
#define MKS_SERVO_RX_PIN 18
#endif

#ifndef MKS_SERVO_BAUD
#define MKS_SERVO_BAUD 256000
#endif

#ifndef LEFT_MOTOR_ID
#define LEFT_MOTOR_ID 0x01
#endif

#ifndef RIGHT_MOTOR_ID
#define RIGHT_MOTOR_ID 0x02
#endif

// =============================================================================
// MOTOR PARAMETERS
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
#define MKS_SERVO_HOLD_PCT abbot::motor::MksServoHoldCurrent::PCT_20
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
#ifndef VELOCITY_MAX_SPEED
#define VELOCITY_MAX_SPEED 3000
#endif

// Default acceleration for speed commands (0-255)
#ifndef MKS_SERVO_ACCEL
#define MKS_SERVO_ACCEL 255
#endif

// Read encoder interval (ms) - Throttled to keep bus clear for commands
#ifndef MKS_SERVO_ENCODER_READ_MS
#define MKS_SERVO_ENCODER_READ_MS 20
#endif
