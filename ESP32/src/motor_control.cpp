// motor_control.cpp
// Thin compatibility wrapper that forwards to the canonical motor driver API.
#include "motor_control.h"
#include "motor_driver.h"
#include "logging.h"

namespace abbot {
namespace motor_control {

void disableMotors() {
  abbot::motor::disableMotors();
  LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "motor_control: forwarded disableMotors() to motor driver");
}

void enableMotors() {
  abbot::motor::enableMotors();
  LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "motor_control: forwarded enableMotors() to motor driver");
}

bool areMotorsEnabled() {
  return abbot::motor::areMotorsEnabled();
}

} // namespace motor_control
} // namespace abbot
