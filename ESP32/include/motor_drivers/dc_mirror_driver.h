// dc_mirror_driver.h
#pragma once

#include "AbstractMotorDriver.h"
#include "motor_drivers/speed_estimator.h"

namespace abbot {
namespace motor {

class DCMirrorDriver : public AbstractMotorDriver {
public:
  /**
   * @brief Construct a new DCMirrorDriver object.
   */
  DCMirrorDriver();

  /**
   * @brief Destroy the DCMirrorDriver object.
   */
  virtual ~DCMirrorDriver() {}

  /**
   * @brief Initialize the DC motor driver hardware and state.
   */
  void initMotorDriver() override;

  /**
   * @brief Clear internal command and target state.
   */
  void clearCommandState() override;

  /**
   * @brief Get the last normalized command sent to a motor.
   * @param side Motor side to query.
   * @return float Normalized command [-1.0..1.0].
   */
  float getLastMotorCommand(MotorSide side) override;

  /**
   * @brief Enable torque/power to the motors.
   */
  void enableMotors() override;

  /**
   * @brief Disable torque/power to the motors (safe state).
   */
  void disableMotors() override;

  /**
   * @brief Check if motors are currently enabled.
   * @return true if enabled, false otherwise.
   */
  bool areMotorsEnabled() override;

  /**
   * @brief Print driver status to the log.
   */
  void printStatus() override;

  /**
   * @brief Dump detailed driver configuration to the log.
   */
  void dumpConfig() override;

  /**
   * @brief Set normalized commands for both motors simultaneously.
   * @param left_command Normalized command for left motor [-1.0..1.0].
   * @param right_command Normalized command for right motor [-1.0..1.0].
   */
  void setMotorCommandBoth(float left_command, float right_command) override;

  /**
   * @brief Set normalized command for a single motor.
   * @param side Motor side to command.
   * @param command Normalized command [-1.0..1.0].
   */
  void setMotorCommand(MotorSide side, float command) override;

  /**
   * @brief Set raw speed command for a single motor.
   * @param side Motor side to command.
   * @param rawSpeed Raw speed units.
   */
  void setMotorCommandRaw(MotorSide side, int16_t rawSpeed) override;

  /**
   * @brief Read the current logical encoder position for a side.
   * @param side Motor side to query.
   * @return int32_t Logical position (respects inversion).
   */
  int32_t readEncoder(MotorSide side) override;

  /**
   * @brief Reset encoder position tracking to zero.
   */
  void resetPositionTracking() override;

  /**
   * @brief Process a driver-specific serial command.
   * @param line The command line string.
   * @return true if the command was handled, false otherwise.
   */
  bool processSerialCommand(const String &line) override;

  /**
   * @brief Get the hardware ID for a motor side.
   * @param side Motor side to query.
   * @return int Hardware ID.
   */
  int getMotorId(MotorSide side) const override;

  /**
   * @brief Check if a motor side is physically inverted.
   * @param side Motor side to query.
   * @return true if inverted, false otherwise.
   */
  bool isMotorInverted(MotorSide side) const override;

  /**
   * @brief Get the maximum speed in raw units.
   * @return float Max speed.
   */
  float getVelocityMaxSpeed() const override;

  /**
   * @brief Get the scale factor for target increments.
   * @return float Increment scale.
   */
  float getVelocityTargetIncrementScale() const override;

  /**
   * @brief Get the position loop proportional gain (if applicable).
   * @return float Kp value.
   */
  float getVelocityPositionKp() const override;

  /**
   * @brief Get the descriptive name of the driver.
   * @return const char* Driver name.
   */
  const char *getDriverName() const override;

  /**
   * @brief Provide a filtered speed estimate in counts/sec.
   * @param side Motor side to query.
   * @return float Speed in counts/sec.
   */
  float readSpeed(MotorSide side) override;

  /**
   * @brief Reset internal speed estimator state.
   */
  void resetSpeedEstimator() override;

  /**
   * @brief Return timestamp (microseconds) when the last command was applied.
   *
   * Drivers store the last command application time in microseconds. Reads and
   * writes of the underlying 64-bit timestamp are protected internally to
   * avoid torn accesses on 32-bit architectures. Callers may compare the
   * returned value to telemetry timestamps to estimate latency; a return value
   * of 0 indicates the driver has not recorded a timestamp.
   *
   * @param side Motor side to query (LEFT or RIGHT).
   * @return uint64_t Timestamp in microseconds, or 0 if unavailable.
   */
  uint64_t getLastCommandTimeUs(MotorSide side) const override;

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
  // Use SpeedEstimator for filtered speed estimates (single-responsibility)
  abbot::motor::SpeedEstimator m_left_estimator{0.25f};
  abbot::motor::SpeedEstimator m_right_estimator{0.25f};
  // Timestamp (microseconds) when the last hardware command was applied
  uint64_t m_last_left_command_time_us = 0;
  uint64_t m_last_right_command_time_us = 0;
  // Protect the 64-bit command timestamps on 32-bit MCUs. Mutable so const
  // getters can safely enter the critical section. For host-side unit tests
  // `portMUX_TYPE` may be unavailable, so use a dummy member under
  // `UNIT_TEST_HOST` to keep headers test-friendly.
#if defined(UNIT_TEST_HOST)
  mutable int m_command_time_mux_dummy = 0;
#else
  mutable portMUX_TYPE m_command_time_mux = portMUX_INITIALIZER_UNLOCKED;
#endif

public:
  // Increment helpers used by ISR integration
  void incrementLeftEncoder(int delta);
  void incrementRightEncoder(int delta);

private:
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

#if defined(UNIT_TEST_HOST)
// Host-side no-op: when running unit tests we may not link the full
// DC driver implementation. Provide a lightweight inline stub so test
// binaries don't require the firmware-only implementation.
inline void installDefaultDCMirrorDriver() {}
#endif

} // namespace motor
} // namespace abbot
