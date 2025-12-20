// motor_control.h
#pragma once

namespace abbot {
namespace motor_control {

// Stub motor control API. Replace implementation to control real motors
// (SCServo).
void disableMotors();
void enableMotors();
bool areMotorsEnabled();

} // namespace motor_control
} // namespace abbot
