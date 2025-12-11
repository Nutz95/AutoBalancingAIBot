// servo_adapter.cpp
// Adapter implementing IMotorDriver by delegating to the existing
// servo-based functions in `abbot::motor::*`.

#include "../include/motor_drivers/IMotorDriver.h"
#include "../include/motor_drivers/driver_manager.h"
#include "motor_drivers/servo_motor_driver.h"

namespace abbot {
namespace motor {

class ServoAdapter : public IMotorDriver {
public:
  void initMotorDriver() override { ::abbot::motor::initMotorDriver(); }
  void clearCommandState() override { ::abbot::motor::clearCommandState(); }
  float getLastMotorCommand(int id) override { return ::abbot::motor::getLastMotorCommand(id); }
  void enableMotors() override { ::abbot::motor::enableMotors(); }
  void disableMotors() override { ::abbot::motor::disableMotors(); }
  bool areMotorsEnabled() override { return ::abbot::motor::areMotorsEnabled(); }
  void printStatus() override { ::abbot::motor::printStatus(); }
  void dumpConfig() override { ::abbot::motor::dumpConfig(); }
  void setMotorCommandBoth(float l, float r) override { ::abbot::motor::setMotorCommandBoth(l, r); }
  void setMotorCommand(int id, float cmd) override { ::abbot::motor::setMotorCommand(id, cmd); }
  void setMotorCommandRaw(int id, int16_t raw) override { ::abbot::motor::setMotorCommandRaw(id, raw); }
  int32_t readEncoder(int id) override { return ::abbot::motor::readEncoder(id); }
  // 64-bit position helpers are optional; use readEncoder/resetPositionTracking
  // (no-op) 64-bit helpers are not part of the common interface
  void resetPositionTracking() override { ::abbot::motor::resetPositionTracking(); }
  bool processSerialCommand(const String &line) override { return ::abbot::motor::processSerialCommand(line); }
};

static ServoAdapter g_servoAdapter;

void installDefaultServoAdapter() {
  setActiveMotorDriver(&g_servoAdapter);
}

} // namespace motor
} // namespace abbot
