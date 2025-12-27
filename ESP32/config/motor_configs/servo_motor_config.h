// servo_motor_config.h -- moved from motor_config.h into motor_configs/
#pragma once

// =============================================================================
// MOTOR IDs AND DIRECTION
// =============================================================================

// Servo-specific defaults (IDs, inversion and velocity mapping used by
// the SCServo-based driver and controller tuning). These are intentionally
// servo-specific and should remain here.

// Default servo IDs (can be overridden by editing this header)
#ifndef LEFT_MOTOR_ID
#define LEFT_MOTOR_ID 8
#endif
#ifndef RIGHT_MOTOR_ID
#define RIGHT_MOTOR_ID 7
#endif

// Per-motor direction inversion (set to 1 if the physical motor is inverted)
// For differential drive balancing: motors must rotate opposite directions.
// WARNING: Motor directions have been validated with MOTOR VEL LEFT/RIGHT 500 commands.
#ifndef LEFT_MOTOR_INVERT
#define LEFT_MOTOR_INVERT 0
#endif
#ifndef RIGHT_MOTOR_INVERT
#define RIGHT_MOTOR_INVERT 0
#endif

// VELOCITY CLOSED-LOOP constants (servo-specific mapping)
#ifndef VELOCITY_MAX_SPEED
#define VELOCITY_MAX_SPEED 7000
#endif

#ifndef VELOCITY_TARGET_INCREMENT_SCALE
#define VELOCITY_TARGET_INCREMENT_SCALE 100.0f
#endif

#ifndef VELOCITY_POSITION_KP
#define VELOCITY_POSITION_KP 10.0f
#endif

// =============================================================================
// SERIAL / UART CONFIGURATION (servo-specific)
// =============================================================================

// Serial pins for SCServo bus
#define SC_SERVO_TX_PIN 17
#define SC_SERVO_RX_PIN 18

// Servo comms baudrate (Feetech STS3215HS supports 1 Mbps)
#define SC_SERVO_BAUD 1000000

// Inter-command delay in microseconds to prevent UART collisions
#ifndef MOTOR_INTER_COMMAND_DELAY_US
#define MOTOR_INTER_COMMAND_DELAY_US 10
#endif

// =============================================================================
// COMPILE-TIME OPTIONS
// =============================================================================

// Compile-time switch: 0 => stub mode, 1 => use real SCServo integration
#ifndef MOTOR_DRIVER_REAL
#define MOTOR_DRIVER_REAL 1
#endif

// Option: perform a brief soft-reset of the servo UART on boot to clear
// stuck commands that may remain across ESP reboots.
#ifndef MOTOR_SERVO_RESET_ON_BOOT
#define MOTOR_SERVO_RESET_ON_BOOT 1
#endif


// =============================================================================
// VELOCITY CLOSED-LOOP MODE (ESP-side position control)
// =============================================================================

#ifndef SERVO_ENCODER_RESOLUTION
#define SERVO_ENCODER_RESOLUTION 4096
#endif

#ifndef VELOCITY_MAX_SPEED
#define VELOCITY_MAX_SPEED 7000
#endif

#ifndef VELOCITY_TARGET_INCREMENT_SCALE
#define VELOCITY_TARGET_INCREMENT_SCALE 100.0f
#endif

#ifndef VELOCITY_POSITION_KP
#define VELOCITY_POSITION_KP 10.0f
#endif

// =============================================================================
// DC MIRROR MODE CONFIG
// =============================================================================

// Note: DC-specific settings were intentionally moved to
// `ESP32/config/motor_configs/dc_motor_config.h` to avoid coupling DC driver
// hardware settings with servo-specific configuration. If you depend on DC
// defines, include that file directly where needed.

