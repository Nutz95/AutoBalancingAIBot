// servo_adapter.cpp
// Adapter implementing IMotorDriver by delegating to the existing
// servo-based functions in `abbot::motor::*`.

#include "../../config/motor_configs/servo_motor_config.h"
#include "../include/motor_drivers/AbstractMotorDriver.h"
#include "../include/motor_drivers/driver_manager.h"
#include "motor_drivers/servo_motor_driver.h"
#include <freertos/portmacro.h>

#include <Arduino.h>

// Protect access to the adapter's speed estimator state when called from
// multiple RTOS tasks. Use a module-local portMUX for minimal overhead.
static portMUX_TYPE s_servo_speed_mux = portMUX_INITIALIZER_UNLOCKED;

// Shared RAII guard for portMUX critical sections. The header documents
// that the guard stores a pointer only; the static mux above therefore
// has sufficient lifetime for this use.
#include "motor_drivers/critical_guard.h"

namespace {
/**
 * @brief IIR alpha for the servo speed estimator (translation-unit scope).
 *
 * This constant is intentionally internal to the implementation; move it
 * to a config header if you need run-time or build-time tuning access.
 */
static constexpr float SERVO_SPEED_IIR_ALPHA = 0.25f;
} // namespace
#include "motor_drivers/speed_estimator.h"
#include <Arduino.h>

namespace abbot {
namespace motor {

class ServoAdapter : public AbstractMotorDriver {
public:
  ServoAdapter() {
    m_est[0].reset();
    m_est[1].reset();
  }

  void initMotorDriver() override { ::abbot::motor::initMotorDriver(); }
  void clearCommandState() override { ::abbot::motor::clearCommandState(); }
  float getLastMotorCommand(MotorSide side) override {
    int id = (side == MotorSide::LEFT) ? LEFT_MOTOR_ID : RIGHT_MOTOR_ID;
    return ::abbot::motor::getLastMotorCommand(id);
  }
  void enableMotors() override { ::abbot::motor::enableMotors(); }
  void disableMotors() override { ::abbot::motor::disableMotors(); }
  bool areMotorsEnabled() override {
    return ::abbot::motor::areMotorsEnabled();
  }
  void printStatus() override { ::abbot::motor::printStatus(); }
  void dumpConfig() override { ::abbot::motor::dumpConfig(); }
  void setMotorCommandBoth(float l, float r) override {
    ::abbot::motor::setMotorCommandBoth(l, r);
  }
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
  void resetPositionTracking() override {
    ::abbot::motor::resetPositionTracking();
  }

  // Configuration/query implementations mapped from servo compile-time macros
  int getMotorId(MotorSide side) const override {
    return (side == MotorSide::LEFT) ? LEFT_MOTOR_ID : RIGHT_MOTOR_ID;
  }

  bool isMotorInverted(MotorSide side) const override {
    return (side == MotorSide::LEFT) ? (LEFT_MOTOR_INVERT != 0)
                                     : (RIGHT_MOTOR_INVERT != 0);
  }

  float getVelocityMaxSpeed() const override {
    return (float)VELOCITY_MAX_SPEED;
  }
  float getVelocityTargetIncrementScale() const override {
    return (float)VELOCITY_TARGET_INCREMENT_SCALE;
  }
  float getVelocityPositionKp() const override {
    return (float)VELOCITY_POSITION_KP;
  }

  const char *getDriverName() const override { return "servo"; }
  // Estimate speed based on accumulated encoder position deltas.
  float readSpeed(MotorSide side) override {
    int id = (side == MotorSide::LEFT) ? LEFT_MOTOR_ID : RIGHT_MOTOR_ID;
    uint32_t now = micros();
    int idx = (side == MotorSide::LEFT) ? 0 : 1;
    int32_t cur = ::abbot::motor::readEncoder(id);
    int64_t curCount = (int64_t)cur;
    // Protect update/read of the estimator state using RAII guard
    {
      abbot::motor::CriticalGuard g(&s_servo_speed_mux);
      float v = m_est[idx].update(curCount, now);
      return v;
    }
  }
private:
  abbot::motor::SpeedEstimator m_est[2];
};

static ServoAdapter g_servoAdapter;

void installDefaultServoAdapter() { setActiveMotorDriver(&g_servoAdapter); }

} // namespace motor
} // namespace abbot
