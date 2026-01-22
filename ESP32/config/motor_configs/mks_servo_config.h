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

// Alternate baud used for scan fallback when the default baud fails.
#ifndef MKS_SERVO_SCAN_ALT_BAUD
#define MKS_SERVO_SCAN_ALT_BAUD 38400
#endif

// =============================================================================
// STEP/DIR (HYBRID MODE)
// =============================================================================
// If enabled, speed commands are sent via pulses (Step/Dir) while RS485 
// is still used for configuration, torque enable, and encoder telemetry.
#ifndef MKS_SERVO_USE_STEP_DIR
#define MKS_SERVO_USE_STEP_DIR 1
#endif

/**
 * @brief Choice of hardware peripheral for generating Step pulses.
 * 0: MCPWM (Motor Control Pulse Width Modulator) - High performance, high freq.
 * 1: RMT (Remote Control) - Excellent for low frequencies, very stable. (Recommended)
 */
#ifndef MKS_SERVO_STEP_GENERATOR_TYPE
#define MKS_SERVO_STEP_GENERATOR_TYPE 1
#endif

// Common Anode wiring (COM tied to 3.3V). 
// Set to 1 if using the wiring from the MKS manual (sinking current).
#ifndef MKS_SERVO_COMMON_ANODE
#define MKS_SERVO_COMMON_ANODE 1
#endif

// Pulse mode configuration (Ensure MKS motor menu matches this!)
// 0 = PULSE + DIRECTION (Recommended)
// 1 = CW + CCW (Not supported by this driver)
#ifndef MKS_SERVO_PULSE_MODE
#define MKS_SERVO_PULSE_MODE 0
#endif

// Echo logging for RS485 (diagnostic)
#ifndef MKS_SERVO_LOG_ECHO
#define MKS_SERVO_LOG_ECHO 1
#endif

// Hex logging for RS485 (diagnostic)
#ifndef MKS_SERVO_LOG_HEX
#define MKS_SERVO_LOG_HEX 1
#endif

// Hardware direction signal inversion
// Set to 1 if motors spin in the opposite direction of the balancer's tilt.
#ifndef MKS_SERVO_PULSE_DIR_INVERT
#define MKS_SERVO_PULSE_DIR_INVERT 0
#endif

// Wiring notes for Step/Dir connection:
// Step : fil vert (Connect to STP/PUL)
// Dir  : fil bleu (Connect to DIR)
// COM  : Connect to 3.3V (for Common Anode)

// Left Motor (P1) Step/Dir pins
// NOTE: GPIO1/2 are UART0 (USB-serial) on ESP32-S3 DevKitC and are not reliable for Step/Dir.
// GPIO4-7 are reserved for SPI in this build, so avoid them too.
// Use free GPIOs to avoid conflicts with the console/bootloader.
#ifndef MKS_SERVO_P1_STEP_PIN
#define MKS_SERVO_P1_STEP_PIN 8 // green
#endif
#ifndef MKS_SERVO_P1_DIR_PIN
#define MKS_SERVO_P1_DIR_PIN 9 // blue
#endif
#ifndef MKS_SERVO_P1_EN_PIN
#define MKS_SERVO_P1_EN_PIN 10 // Same side as STEP/DIR for wiring convenience
#endif

// Right Motor (P2) Step/Dir pins
// Adjacent pins on Left header
#ifndef MKS_SERVO_P2_STEP_PIN
#define MKS_SERVO_P2_STEP_PIN 17 // green
#endif
#ifndef MKS_SERVO_P2_DIR_PIN
#define MKS_SERVO_P2_DIR_PIN 18 // blue
#endif
#ifndef MKS_SERVO_P2_EN_PIN
#define MKS_SERVO_P2_EN_PIN 16 // black
#endif

// Technical Parameters (Timeouts, Deadbands, etc.)
#include "mks_servo_constants.h"

// Enable RS485 telemetry (encoder reading) during balancing.
// WARNING: On some MKS firmwares, Step/Dir pulses can interfere with RS485.
// Set to 0 to prioritize loop stability for balancing V1.
#ifndef MKS_SERVO_TELEMETRY_ENABLED
#define MKS_SERVO_TELEMETRY_ENABLED 1
#endif

// Microsteps per revolution (configured on MKS driver menu)
// Used to calculate frequency of steps from target speed.
#ifndef MKS_SERVO_STEPS_PER_REV
#define MKS_SERVO_STEPS_PER_REV (200 * 32) // 200 pulses * 32 mstep
#endif

// FreeRTOS task priority for each motor bus task.
// Keep this high to preserve command latency during balancing.
#ifndef MKS_SERVO_TASK_PRIORITY
#define MKS_SERVO_TASK_PRIORITY 21
#endif

// Divider applied to the IMU consumer notification rate (typically 1000Hz)
// to reduce motor bus packet rate. Example: 2 => 500Hz.
#ifndef MKS_SERVO_CONTROL_SEND_DIVIDER
#define MKS_SERVO_CONTROL_SEND_DIVIDER 2
#endif

// Limit Step/Dir LEDC reconfiguration rate (Hz). Default 1000Hz.
// Higher values increase CPU usage on core 1.
// Recommended for balancing: 100-250Hz.
#ifndef MKS_SERVO_STEP_UPDATE_HZ
#define MKS_SERVO_STEP_UPDATE_HZ 125
#endif

// Throttle frequency updates: only update if change is > X Hz
// NOTE: Both motors are on SEPARATE RS485 buses (Left=Serial2, Right=Serial1).
// Each bus has dedicated bandwidth. IDs can be different or identical - doesn't matter
// since buses are independent. DO NOT CHANGE these IDs without user authorization.
#ifndef LEFT_MOTOR_ID
#define LEFT_MOTOR_ID 0x01
#endif

#ifndef RIGHT_MOTOR_ID
#define RIGHT_MOTOR_ID 0x01
#endif

// =============================================================================
// MOTOR PARAMETERS
// =============================================================================

// =============================================================================
// MOTOR PARAMETERS (!!! DO NOT CHANGE INVERSION SETTINGS WITHOUT NOTIFYING USER !!!)
// These are calibrated for physical wiring.
// =============================================================================

// Default operating mode (CR_vFOC recommended for Step/Dir hybrid)
// 0=CR_OPEN, 1=CR_CLOSE, 2=CR_vFOC (Pulse control)
// 3=SR_OPEN, 4=SR_CLOSE, 5=SR_vFOC (Serial control)
#ifndef MKS_SERVO_DEFAULT_MODE
#define MKS_SERVO_DEFAULT_MODE abbot::motor::MksServoMode::CR_vFOC
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

// Interpolation Smoothing Time Constant (seconds)
// Time taken for encoder "jumps" to bleed out. 
// 0.010f (10ms) is recommended for 200Hz telemetry.
#ifndef MKS_SERVO_INTERP_TAU_S
#define MKS_SERVO_INTERP_TAU_S 0.010f
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


// Enable/disable RS485 encoder telemetry (0x31) for A/B testing.
#ifndef MKS_SERVO_TELEMETRY_ENABLED
#define MKS_SERVO_TELEMETRY_ENABLED 1
#endif

// Default acceleration for speed commands (0-255)
#ifndef MKS_SERVO_ACCEL
#define MKS_SERVO_ACCEL 255
#endif

// Velocity estimator alpha (Smoothing). 
// 0.05 at 100Hz = very smooth. Lowering this stops LQR "chatter".
#ifndef MKS_SERVO_SPEED_ALPHA
#define MKS_SERVO_SPEED_ALPHA 0.05f      // Reverted to 0.05 for smoother 1000Hz operation
#endif

// Timeout for logging diagnostic errors (ms) to avoid console spam
#ifndef MKS_SERVO_DIAG_LOG_ms
#define MKS_SERVO_DIAG_LOG_ms 2000
#endif

// Read encoder interval (ms) - Throttled to keep bus clear for commands
// Minimum time between encoder reads (milliseconds) to avoid bus saturation.
// Increased to 100ms to maximize bandwidth for balancing at 100Hz.
// Interval for telemetry push from motor in Hybrid mode.
// 20ms (50Hz) is recommended for stable balancing.
#ifndef MKS_SERVO_ENCODER_READ_MS
#define MKS_SERVO_ENCODER_READ_MS 5 // 200Hz
#endif

// Max time (ms) to extrapolate encoder position between telemetry frames.
// This smoothing (first-order hold) removes "staircase" artifacts for LQR.
// Recommended: 2-3x the MKS_SERVO_ENCODER_READ_MS value.
#ifndef MKS_SERVO_ENCODER_EXTRAPOLATE_MS
#define MKS_SERVO_ENCODER_EXTRAPOLATE_MS 15
#endif

// Periodic telemetry interval (ms) when using automatic upload (command 0x01).
// This value should remain modest to avoid saturating the bus.
#ifndef MKS_SERVO_PERIODIC_TELEMETRY_MS
#define MKS_SERVO_PERIODIC_TELEMETRY_MS MKS_SERVO_ENCODER_READ_MS
#endif

// =============================================================================
// COMMUNICATION TIMING (Calculated based on MKS_SERVO_BAUD)
// =============================================================================
// Formula: (bits / baud) * 1,000,000 + margin
// For 256000 bps, 1 bit = 3.9us. 
// A 10-byte transaction (100 bits) takes ~390us raw wire time.
// Motor response delay (internal processing) is typically 200-500us.

// Critical command timeout (e.g. speed command)
// Reduced to 1000us to ensure we don't break the 500Hz budget.
#ifndef MKS_SERVO_TIMEOUT_CONTROL_US
#define MKS_SERVO_TIMEOUT_CONTROL_US 1000
#endif

// Telemetry/Data timeout (e.g. reading position)
// Position reads can take longer to process internally on MKS drivers.
// Reduced to 2000us (2ms) for live balancing safety.
#ifndef MKS_SERVO_TIMEOUT_DATA_US
#define MKS_SERVO_TIMEOUT_DATA_US 2000 
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

// Optional quiet window (microseconds) for Hybrid Step/Dir mode.
// Some MKS firmwares stop responding to RS485 telemetry (0x31) while receiving
// high-frequency Step/Dir pulses. Setting a small quiet window temporarily
// disables step pulses around the telemetry request to allow the driver to
// respond. 
// WARNING: Setting this > 0 will stop the motors briefly during telemetry,
// which can destabilize the balancer. Use only if necessary for diagnostics.
#ifndef MKS_SERVO_TELEMETRY_QUIET_WINDOW_US
#define MKS_SERVO_TELEMETRY_QUIET_WINDOW_US 0
#endif

// Turnaround delay for RS485 auto-direction hardware (us)
#ifndef MKS_SERVO_RS485_TURNAROUND_US
#define MKS_SERVO_RS485_TURNAROUND_US 200
#endif

// How long to wait for bus availability (ms)
#ifndef MKS_SERVO_BUS_MUTEX_TIMEOUT_MS
#define MKS_SERVO_BUS_MUTEX_TIMEOUT_MS 20
#endif
