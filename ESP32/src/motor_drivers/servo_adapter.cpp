// servo_adapter.cpp
// Adapter implementing IMotorDriver by delegating to the existing
// servo-based functions in `abbot::motor::*`.

#include "../include/motor_drivers/AbstractMotorDriver.h"
#include "../include/motor_drivers/driver_manager.h"
#include "motor_drivers/servo_motor_driver.h"
#include "../../config/motor_configs/servo_motor_config.h"

namespace abbot {
namespace motor {

class ServoAdapter : public AbstractMotorDriver {
public:
  void initMotorDriver() override { ::abbot::motor::initMotorDriver(); }
  void clearCommandState() override { ::abbot::motor::clearCommandState(); }
  float getLastMotorCommand(MotorSide side) override {
    int id = (side == MotorSide::LEFT) ? LEFT_MOTOR_ID : RIGHT_MOTOR_ID;
    return ::abbot::motor::getLastMotorCommand(id);
  }
  void enableMotors() override { ::abbot::motor::enableMotors(); }
  void disableMotors() override { ::abbot::motor::disableMotors(); }
  bool areMotorsEnabled() override { return ::abbot::motor::areMotorsEnabled(); }
  void printStatus() override { ::abbot::motor::printStatus(); }
  void dumpConfig() override { ::abbot::motor::dumpConfig(); }
  void setMotorCommandBoth(float l, float r) override { ::abbot::motor::setMotorCommandBoth(l, r); }
  void setMotorCommand(MotorSide side, float cmd) override {
    int id = (side == MotorSide::LEFT) ? LEFT_MOTOR_ID : RIGHT_MOTOR_ID;
    ::abbot::motor::setMotorCommand(id, cmd);
  }
  void setMotorCommandRaw(MotorSide side, int16_t raw) override {
    int id = (side == MotorSide::LEFT) ? LEFT_MOTOR_ID : RIGHT_MOTOR_ID;
    ::abbot::motor::setMotorCommandRaw(id, raw);
  }
  int32_t readEncoder(MotorSide side) override {
    int id = (side == MotorSide::LEFT) ? LEFT_MOTOR_ID : RIGHT_MOTOR_ID;
    return ::abbot::motor::readEncoder(id);
  }
  // 64-bit position helpers are optional; use readEncoder/resetPositionTracking
  // (no-op) 64-bit helpers are not part of the common interface
  void resetPositionTracking() override { ::abbot::motor::resetPositionTracking(); }

  // Configuration/query implementations mapped from servo compile-time macros
  int getMotorId(MotorSide side) const override {
    return (side == MotorSide::LEFT) ? LEFT_MOTOR_ID : RIGHT_MOTOR_ID;
  }

  bool isMotorInverted(MotorSide side) const override {
    return (side == MotorSide::LEFT) ? (LEFT_MOTOR_INVERT != 0) : (RIGHT_MOTOR_INVERT != 0);
  }

  float getVelocityMaxSpeed() const override { return (float)VELOCITY_MAX_SPEED; }
  float getVelocityTargetIncrementScale() const override { return (float)VELOCITY_TARGET_INCREMENT_SCALE; }
  float getVelocityPositionKp() const override { return (float)VELOCITY_POSITION_KP; }

  const char* getDriverName() const override { return "servo"; }
};

static ServoAdapter g_servoAdapter;

void installDefaultServoAdapter() {
  setActiveMotorDriver(&g_servoAdapter);
}

} // namespace motor
} // namespace abbot
