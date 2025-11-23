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
#define SC_SERVO_TX_PIN 17
#define SC_SERVO_RX_PIN 18

// Servo comms baudrate (Feetech STS3215HS supports 1 Mbps)
#define SC_SERVO_BAUD 1000000

// Maximum servo speed units used when mapping normalized commands to servo
// speed units. Many ST/Feetech firmwares accept larger values; adjust to
// match your hardware. Default set to 4000 as a safe higher-speed limit.
#ifndef SC_SERVO_MAX_SPEED
#define SC_SERVO_MAX_SPEED 7000
#endif

// Compile-time switch: 0 => stub mode, 1 => use real `workloads/SCServo` integration
#ifndef MOTOR_DRIVER_REAL
// Enable the real motor driver integration (workloads/SCServo / URT-1 transport)
// WARNING: when enabled this will actually send commands to servos. Keep motors
// mechanically restrained and be ready to cut power when testing on bench.
#define MOTOR_DRIVER_REAL 1
#endif

// Motor control mode selection for real servos. Choose one of:
//  - MOTOR_CONTROL_POSITION: map normalized command to servo position (closed-loop servo mode)
//  - MOTOR_CONTROL_VELOCITY: map normalized command to a velocity-like command (if supported)
//  - MOTOR_CONTROL_MOTOR: low-level motor control mode (URT-1 / motor output)
#define MOTOR_CONTROL_POSITION 0
#define MOTOR_CONTROL_VELOCITY 1
#define MOTOR_CONTROL_MOTOR 2

#ifndef MOTOR_CONTROL_MODE
// Default control mode for balancing: VELOCITY control maps normalized commands
// from the controller to servo motor speed setpoints.
#define MOTOR_CONTROL_MODE MOTOR_CONTROL_VELOCITY
#endif

// Position mapping defaults (only used for POSITION control):
#ifndef MOTOR_POSITION_CENTER
#define MOTOR_POSITION_CENTER 2048
#endif
#ifndef MOTOR_POSITION_RANGE
#define MOTOR_POSITION_RANGE 1000
#endif
