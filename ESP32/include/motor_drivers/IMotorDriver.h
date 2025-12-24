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
  /**
   * @brief Read estimated velocity for the given motor side.
   *
   * The value is expressed in encoder counts per second (counts/s).
   * Implementations should return a filtered/averaged estimate where
   * practical to reduce jitter from raw counts. A return value of
   * 0.0f indicates either no movement or that the driver cannot provide
   * an estimate.
   *
   * @param side Which motor side to query (`MotorSide::LEFT` or `RIGHT`).
   * @return float Estimated speed in counts per second (counts/s).
   *
   * @note Callers that rely on the estimate should consider small values
   * near 0 as potentially uninitialized and apply validation or deadband.
   *
   * @threadsafe The thread-safety of this method depends on the driver
   * implementation. Driver implementations must document their
   * concurrency guarantees. `driver_manager` wrappers may call this from
   * console tasks or RTOS threads; the caller should not assume ISR safety.
   */
  virtual float readSpeed(MotorSide side) = 0;
  // Reset any internal speed estimator state so subsequent readSpeed() calls
  // return a fresh estimate (useful after a prolonged idle or when enabling
  // telemetry). Implementations should reset both side estimators.
  virtual void resetSpeedEstimator() = 0;
  // Return the timestamp (microseconds) when the last command was applied
  // to the given side. Implementations should update this when a command
  // is actually written to hardware so telemetry can correlate latency.
  /**
   * @brief Return the timestamp (microseconds) when the last command was
   * applied to the given side.
   *
   * Implementations should update this value at the point the command is
   * actually written to hardware (e.g. right before toggling an enable pin
   * or applying PWM duty) so host-side telemetry can correlate command-to-
   * measurement latency. Drivers that do not support timestamping should
   * return 0.
   *
   * Thread-safety: on 32-bit MCUs a 64-bit timestamp may require protection
   * to avoid torn reads/writes. Drivers MUST document their concurrency
   * guarantees and protect 64-bit accesses (for example using a mutex or
   * critical section). Callers should NOT assume ISR-safety unless the
   * driver explicitly documents it.
   *
   * @param side Motor side to query (`MotorSide::LEFT` or `RIGHT`).
   * @return uint64_t Timestamp in microseconds, or 0 if unavailable.
   */
  virtual uint64_t getLastCommandTimeUs(MotorSide side) const = 0;
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
