// dc_mirror_driver.h
#pragma once

#include "IMotorDriver.h"

namespace abbot {
namespace motor {

class DCMirrorDriver : public IMotorDriver {
public:
  DCMirrorDriver();
  virtual ~DCMirrorDriver() {}
  void initMotorDriver() override;
  void clearCommandState() override;
  float getLastMotorCommand(MotorSide side) override;
  void enableMotors() override;
  void disableMotors() override;
  bool areMotorsEnabled() override;
  void printStatus() override;
  void dumpConfig() override;
  void setMotorCommandBoth(float left_command, float right_command) override;
  void setMotorCommand(MotorSide side, float command) override;
  void setMotorCommandRaw(MotorSide side, int16_t rawSpeed) override;
  int32_t readEncoder(MotorSide side) override;
  void resetPositionTracking() override;
  bool processSerialCommand(const String &line) override;

  // Configuration/query API (implements the IMotorDriver queries)
  int getMotorId(MotorSide side) const override;
  bool isMotorInverted(MotorSide side) const override;
  float getVelocityMaxSpeed() const override;
  float getVelocityTargetIncrementScale() const override;
  float getVelocityPositionKp() const override;
  const char* getDriverName() const override;

private:
  void applyMirrorIfNeeded();
  void checkDivergenceAndSafety();

  bool m_enabled;
  float m_last_left_cmd;
  float m_last_right_cmd;
  int64_t m_left_encoder;
  int64_t m_right_encoder;
};

// Factory helper: create and install DC mirror driver as active driver
void installDefaultDCMirrorDriver();

} // namespace motor
} // namespace abbot
