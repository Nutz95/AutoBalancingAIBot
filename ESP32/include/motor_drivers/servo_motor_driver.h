// servo_motor_driver.h
#pragma once

#include <Arduino.h>
#include <stdint.h>

namespace abbot {
namespace motor {

// Initialize motor driver (called at boot). Safe to call multiple times.
void initMotorDriver();

// Enable/disable motors (disable = safe state; default at boot = disabled)
// enableMotors() switches servos to WheelMode for velocity control
void enableMotors();
void disableMotors();
bool areMotorsEnabled();

// Reset knowledge of the last commanded velocities/positions so that when
// the servos are re-enabled we start from zero rather than resuming the
// previous command state.
void clearCommandState();

// Send a normalized motor command in range [-1.0, +1.0]
// In VELOCITY_CLOSED_LOOP mode, command is converted to position increment
// and ESP computes velocity from position error (closed-loop control)
void setMotorCommand(int id, float command);

// Send normalized commands to LEFT and RIGHT motors in a single synchronized
// call. Commands are clamped to [-1,1], inversion applied per motor. Position
// targets are updated and velocity commands computed from position error.
void setMotorCommandBoth(float left_command, float right_command);

// Read last sent normalized motor command for an id. If never set, returns 0.0f
float getLastMotorCommand(int id);

// Send a raw servo-speed command directly (servo units, signed). This bypasses
// normalization and sends the value as-is to the servo's speed register.
void setMotorCommandRaw(int id, int16_t rawSpeed);

// Read accumulated encoder position with unwrap (unlimited range).
// Returns int64_t position in encoder counts (4096 per revolution).
int32_t readEncoder(int id);

// Trigger a hardware refresh of the encoder position for the given ID.
// This performs the serial UART transaction with the servo.
bool refreshEncoder(int id);

// Reset position tracking: re-initialize accumulators to current encoder
// positions and clear all position targets. Use to recover from any drift or
// desync.
void resetPositionTracking();

// Process a single-line serial command for motor driver. Returns true if the
// command was recognized and handled.
// Commands: DUMP, RESETPOS, POS
bool processSerialCommand(const String &line);

// Print current motor status information to the motor log channel.
void printStatus();

// Dump motor config (IDs, pins) to the motor log channel.
void dumpConfig();

} // namespace motor
} // namespace abbot
