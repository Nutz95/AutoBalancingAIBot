// motor_control.cpp (stub implementation)
#include "motor_control.h"
#include <Arduino.h>

namespace abbot {
namespace motor_control {

static bool g_enabled = false;

void disableMotors() {
  // Default stub: just flip flag. Replace with SCServo calls if available.
  g_enabled = false;
  Serial.println("motor_control: motors disabled (stub)");
}

void enableMotors() {
  g_enabled = true;
  Serial.println("motor_control: motors enabled (stub)");
}

bool areMotorsEnabled() { return g_enabled; }

} // namespace motor_control
} // namespace abbot
