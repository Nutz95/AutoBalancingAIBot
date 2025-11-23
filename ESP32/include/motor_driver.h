// motor_driver.h
#pragma once

#include <stdint.h>
#include <Arduino.h>

namespace abbot {
namespace motor {

// Initialize motor driver (called at boot). Safe to call multiple times.
void initMotorDriver();

// Enable/disable motors (disable = safe state; default at boot = disabled)
void enableMotors();
void disableMotors();
bool areMotorsEnabled();

// Send a normalized motor command in range [-1.0, +1.0]
// `id` must be a motor ID (e.g., LEFT_MOTOR_ID / RIGHT_MOTOR_ID)
void setMotorCommand(int id, float command);

// Send a raw servo-speed command directly (servo units, signed). This bypasses
// normalization and sends the value as-is to the servo's speed register.
void setMotorCommandRaw(int id, int16_t rawSpeed);

// Read encoder / position feedback. Returns an integer position or 0 if not available.
int32_t readEncoder(int id);

// Process a single-line serial command for motor driver. Returns true if the
// command was recognized and handled.
bool processSerialCommand(const String &line);

} // namespace motor
} // namespace abbot
