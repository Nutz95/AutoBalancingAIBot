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
  virtual void initMotorDriver() = 0;
  virtual void clearCommandState() = 0;
  virtual float getLastMotorCommand(int id) = 0;
  virtual void enableMotors() = 0;
  virtual void disableMotors() = 0;
  virtual bool areMotorsEnabled() = 0;
  virtual void printStatus() = 0;
  virtual void dumpConfig() = 0;
  virtual void setMotorCommandBoth(float left_command, float right_command) = 0;
  virtual void setMotorCommand(int id, float command) = 0;
  virtual void setMotorCommandRaw(int id, int16_t rawSpeed) = 0;
  virtual int32_t readEncoder(int id) = 0;
  // Note: if an implementation exposes 64-bit position APIs these may be
  // accessed via driver-specific headers; the common interface exposes
  // `readEncoder` and `resetPositionTracking` for portability.
  virtual void resetPositionTracking() = 0;
  virtual bool processSerialCommand(const String &line) = 0;
};

} // namespace motor
} // namespace abbot
