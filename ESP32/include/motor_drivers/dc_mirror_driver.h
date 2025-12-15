// dc_mirror_driver.h
#pragma once

#include "AbstractMotorDriver.h"

namespace abbot {
namespace motor {

class DCMirrorDriver : public AbstractMotorDriver {
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

  // Hardware control helpers
  void applyHardwareCommand(MotorSide side, int16_t rawSpeed);
  void stopMotorHardware(MotorSide side);
  // Lower-level hardware helpers (atomic, well-named)
  void configurePWMPin(int pin, int chan);
  void configureEnablePin(int pin);
  void setPWMduty(int chan, uint32_t duty);
  void setEnablePinState(int pin, bool high);
  void safeDirectionChangeDelay();
  // Apply PWM duty to the correct channel for a side and ensure EN pins
  void applyDutyForSide(MotorSide side, int32_t desired, uint32_t duty);

  // PWM / enable pin state
  int m_left_pwm_r_pin;
  int m_left_pwm_l_pin;
  int m_left_en_r_pin;
  int m_left_en_l_pin;
  int m_right_pwm_r_pin;
  int m_right_pwm_l_pin;
  int m_right_en_r_pin;
  int m_right_en_l_pin;

  // LEDC channels for PWM (or -1 if not attached)
  int m_left_pwm_r_chan;
  int m_left_pwm_l_chan;
  int m_right_pwm_r_chan;
  int m_right_pwm_l_chan;
  int m_pwm_resolution_bits;
  int m_pwm_max_duty;

  bool m_enabled;
  float m_last_left_cmd;
  float m_last_right_cmd;
  // Last applied direction per side: -1 = neg, 0 = stopped, 1 = pos
  int m_last_left_dir;
  int m_last_right_dir;
  int64_t m_left_encoder;
  int64_t m_right_encoder;
  // Increment helpers used by ISR integration
public:
  void incrementLeftEncoder(int delta);
  void incrementRightEncoder(int delta);
  // PCNT units and configuration flags (set when PCNT is initialized)
  int m_left_pcnt_unit = -1;
  int m_right_pcnt_unit = -1;
  bool m_left_pcnt_configured = false;
  bool m_right_pcnt_configured = false;
  // Runtime inversion overrides (when enabled, take precedence over macros)
  bool m_left_invert_override_enabled;
  bool m_left_invert_override_value;
  bool m_right_invert_override_enabled;
  bool m_right_invert_override_value;
};

// Factory helper: create and install DC mirror driver as active driver
void installDefaultDCMirrorDriver();

} // namespace motor
} // namespace abbot
