// motor_control.cpp
// Thin compatibility wrapper that forwards to the canonical motor driver API.
#include "motor_control.h"
#include "logging.h"
#include "motor_drivers/driver_manager.h"

namespace abbot {
namespace motor_control {

void disableMotors() {
  if (auto drv = abbot::motor::getActiveMotorDriver()) {
    drv->disableMotors();
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT,
                "motor_control: forwarded disableMotors() to motor driver");
  } else {
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT,
                "motor_control: no active motor driver to disable");
  }
}

void enableMotors() {
  if (auto drv = abbot::motor::getActiveMotorDriver()) {
    drv->enableMotors();
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT,
                "motor_control: forwarded enableMotors() to motor driver");
  } else {
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT,
                "motor_control: no active motor driver to enable");
  }
}

bool areMotorsEnabled() {
  if (auto drv = abbot::motor::getActiveMotorDriver())
    return drv->areMotorsEnabled();
  return false;
}

} // namespace motor_control
} // namespace abbot
