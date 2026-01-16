// AbstractMotorDriver.h
#pragma once

#include "IMotorDriver.h"
#include "logging.h"

namespace abbot {
namespace motor {

// Base class that implements common textual serial command parsing for
// motor drivers (MOTOR ENABLE, MOTOR SET, MOTOR READ, etc.). Drivers can
// inherit from this to get the shared parsing while implementing the
// virtual hardware methods declared in IMotorDriver.
class AbstractMotorDriver : public IMotorDriver {
public:
  virtual ~AbstractMotorDriver() {
  }

  // Public API: parse a serial command string. Implementation moved to
  // `src/motor_drivers/abstract_motor_driver.cpp` to avoid heavy header
  // dependencies and improve compile-time isolation.
  bool processSerialCommand(const String &line) override;

  /**
   * Default `readSpeed` implementation for drivers that don't provide
   * a hardware speed estimate. Implemented in the corresponding .cpp to
   * avoid single-line inline bodies in headers.
   */
  virtual float readSpeed(MotorSide /*side*/) override;

  /**
   * Default implementation that reads encoders sequentially.
   * Drivers with high-speed parallel bus support should override this.
   */
  virtual void readEncodersBoth(int32_t &left, int32_t &right) override;

  // Default implementation for drivers that don't track command timestamps.
  virtual uint64_t getLastCommandTimeUs(MotorSide /*side*/) const override;
};

} // namespace motor
} // namespace abbot
