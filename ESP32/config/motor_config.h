// motor_config.h -- motor ID and direction configuration
#pragma once

// =============================================================================
// MOTOR IDs AND DIRECTION
// =============================================================================

// Default servo IDs (can be overridden by editing this header)
#define LEFT_MOTOR_ID 8
#define RIGHT_MOTOR_ID 7

// Per-motor direction inversion (set to 1 if the physical motor is inverted)
// For differential drive balancing: motors must rotate opposite directions.
//
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// WARNING: DO NOT CHANGE THESE VALUES!
// Motor directions have been validated with MOTOR VEL LEFT/RIGHT 500 commands.
// Both motors correctly go FORWARD with positive velocity.
// If balancing direction is wrong, fix it in the PID/balancer code, NOT here!
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
#define LEFT_MOTOR_INVERT 1
#define RIGHT_MOTOR_INVERT 0

// =============================================================================
// SERIAL / UART CONFIGURATION
// =============================================================================

// Serial pins for SCServo bus
#define SC_SERVO_TX_PIN 17
#define SC_SERVO_RX_PIN 18

// Servo comms baudrate (Feetech STS3215HS supports 1 Mbps)
#define SC_SERVO_BAUD 1000000

// Inter-command delay in microseconds to prevent UART collisions
// At 1Mbps, a command frame takes ~300µs. Add margin for reliability.
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
// Servos run in Wheel/Velocity mode. Position tracking is done on ESP32
// using encoder feedback with 4096-count unwrapping.
// This removes the ±32767 multi-turn limit of servo position mode.
// Position is tracked as int64_t = virtually unlimited range.

// Encoder resolution per revolution (STS3215 magnetic encoder)
#ifndef SERVO_ENCODER_RESOLUTION
#define SERVO_ENCODER_RESOLUTION 4096
#endif

// Maximum servo speed units for velocity commands
// STS3215 max is ~3400 steps/s at no load.
#ifndef VELOCITY_MAX_SPEED
#define VELOCITY_MAX_SPEED 7000
#endif

// Velocity command scaling
// How many encoder counts to add to target per cycle per unit command [-1..1]
// At 166Hz control rate, value of 100 means command=1.0 adds 100 counts/cycle
// = 16600 counts/sec ≈ 4 rotations/sec (near max speed of 106 RPM)
#ifndef VELOCITY_TARGET_INCREMENT_SCALE
#define VELOCITY_TARGET_INCREMENT_SCALE 100.0f
#endif

// Position control gain (P controller for velocity closed-loop)
// velocity_cmd = VELOCITY_POSITION_KP * position_error
// Units: speed_units per encoder_count of error
// Higher = more aggressive correction, risk of overshoot
// Lower = smoother but slower response
// For balancing: needs fast response, use 10-20
#ifndef VELOCITY_POSITION_KP
#define VELOCITY_POSITION_KP 10.0f
#endif
