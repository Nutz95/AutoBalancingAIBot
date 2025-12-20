// IMotorDriver.h
#pragma once
#include <stdint.h>
#if defined(UNIT_TEST_HOST)
#include <string>
using String = std::string;
#else
#include <Arduino.h>
#endif

namespace abbot {
namespace motor {

// Abstract interface for motor drivers. Implement this when adding new
// motor types (DC+encoder, stepper, etc.). This interface is intentionally
// small and mirrors the existing top-level motor API so adapters are simple.
class IMotorDriver {
public:
  virtual ~IMotorDriver() {}
  // Motor side selector for per-motor queries
  enum class MotorSide { LEFT = 0, RIGHT = 1 };
  virtual void initMotorDriver() = 0;
  virtual void clearCommandState() = 0;
  // Return the last normalized motor command for the given motor side
  virtual float getLastMotorCommand(MotorSide side) = 0;
  virtual void enableMotors() = 0;
  virtual void disableMotors() = 0;
  virtual bool areMotorsEnabled() = 0;
  virtual void printStatus() = 0;
  virtual void dumpConfig() = 0;
  virtual void setMotorCommandBoth(float left_command, float right_command) = 0;
  // Use MotorSide for per-motor control instead of numeric IDs
  virtual void setMotorCommand(MotorSide side, float command) = 0;
  virtual void setMotorCommandRaw(MotorSide side, int16_t rawSpeed) = 0;
  virtual int32_t readEncoder(MotorSide side) = 0;
  // Note: if an implementation exposes 64-bit position APIs these may be
  // accessed via driver-specific headers; the common interface exposes
  // `readEncoder` and `resetPositionTracking` for portability.
  virtual void resetPositionTracking() = 0;
  virtual bool processSerialCommand(const String &line) = 0;

  // --- Driver-provided configuration/query API ---
  // Motor side selector for per-motor queries
  // (MotorSide already declared above.)

  // Return the motor ID used by this driver for the given side
  virtual int getMotorId(MotorSide side) const = 0;

  // Return true if the motor on the given side is inverted
  virtual bool isMotorInverted(MotorSide side) const = 0;

  // Velocity / closed-loop tuning parameters (drivers may map these from
  // compile-time macros or runtime config). Provide floats for generality.
  virtual float getVelocityMaxSpeed() const = 0;
  virtual float getVelocityTargetIncrementScale() const = 0;
  virtual float getVelocityPositionKp() const = 0;

  // Human-readable driver name (e.g. "servo", "dc_mirror", ...)
  virtual const char *getDriverName() const = 0;
};

} // namespace motor
} // namespace abbot
