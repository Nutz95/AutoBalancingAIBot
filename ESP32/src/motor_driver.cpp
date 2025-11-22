// motor_driver.cpp
#include "motor_driver.h"
#include "../config/motor_config.h"
#include <Arduino.h>

namespace abbot {
namespace motor {

static bool s_motors_enabled = false;

void initMotorDriver() {
  // Ensure safe default
  s_motors_enabled = false;
  Serial.println("motor_driver: init (motors disabled by default)");

#if MOTOR_DRIVER_REAL
  // TODO: initialize SCServo transport here (workloads/SCServo)
  Serial.println("motor_driver: compiled in REAL mode (SCServo). Initializing bus...");
  // Example (pseudo): SCServo.begin(SC_SERVO_BAUD); // depends on library API
#else
  Serial.println("motor_driver: compiled in STUB mode");
#endif
}

void enableMotors() {
  if (s_motors_enabled) return;
  s_motors_enabled = true;
  Serial.println("motor_driver: motors ENABLED");
}

void disableMotors() {
  if (!s_motors_enabled) return;
  s_motors_enabled = false;
  Serial.println("motor_driver: motors DISABLED");
}

bool areMotorsEnabled() {
  return s_motors_enabled;
}

void setMotorCommand(int id, float command) {
  // clamp
  if (command > 1.0f) command = 1.0f;
  if (command < -1.0f) command = -1.0f;

  if (!s_motors_enabled) {
    Serial.print("motor_driver: setMotorCommand ignored (motors disabled) id="); Serial.print(id);
    Serial.print(" cmd="); Serial.println(command, 4);
    return;
  }

  // Apply per-motor inversion from config
  int invert = 0;
  if (id == LEFT_MOTOR_ID) invert = LEFT_MOTOR_INVERT;
  else if (id == RIGHT_MOTOR_ID) invert = RIGHT_MOTOR_INVERT;
  if (invert) command = -command;

#if MOTOR_DRIVER_REAL
  // Map [-1..1] to servo command space. Implementation depends on SCServo API.
  // For now translate to a pseudo-position/velocity and print; replace with real SCServo calls.
  int32_t scaled = (int32_t)(command * 1000); // placeholder
  Serial.print("motor_driver: REAL set id="); Serial.print(id); Serial.print(" val="); Serial.println(scaled);
  // Example real call (uncomment and adapt depending on SCServo API):
  // SCServo.move(id, map_to_servo_angle_or_velocity(scaled));
#else
  Serial.print("motor_driver: STUB set id="); Serial.print(id); Serial.print(" cmd="); Serial.println(command,4);
#endif
}

int32_t readEncoder(int id) {
#if MOTOR_DRIVER_REAL
  // TODO: query SCServo feedback (position) and return it.
  Serial.print("motor_driver: REAL readEncoder id="); Serial.println(id);
  return 0; // placeholder until real API is used
#else
  (void)id;
  return 0;
#endif
}

} // namespace motor
} // namespace abbot
