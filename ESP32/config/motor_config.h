// motor_config.h -- motor ID and direction configuration
#pragma once

// Default servo IDs (can be overridden by editing this header)
#define LEFT_MOTOR_ID 8
#define RIGHT_MOTOR_ID 7

// Per-motor direction inversion (set to 1 if the physical motor is inverted)
#define LEFT_MOTOR_INVERT 1
#define RIGHT_MOTOR_INVERT 0

// Serial pins for SCServo bus (if using software Serial or board-assigned pins)
// Default values documented in openspec/project.md: S_TXD=GPIO19, S_RXD=GPIO18
// Use GPIO18 as default S_RXD and GPIO19 as default S_TXD for the SCServo bus.
#define SC_SERVO_TX_PIN 19
#define SC_SERVO_RX_PIN 18

// Servo comms baudrate (Feetech STS3215HS supports 1 Mbps)
#define SC_SERVO_BAUD 1000000

// Compile-time switch: 0 => stub mode, 1 => use real `workloads/SCServo` integration
#ifndef MOTOR_DRIVER_REAL
#define MOTOR_DRIVER_REAL 0
#endif
