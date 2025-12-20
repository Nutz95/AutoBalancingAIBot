// driver_manager.h
#pragma once
#include "IMotorDriver.h"
#include <cstdint>

namespace abbot {
namespace motor {

// Set the active motor driver implementation. Ownership remains with caller.
void setActiveMotorDriver(IMotorDriver *drv);

// Get the currently active driver (may be nullptr if none set).
IMotorDriver *getActiveMotorDriver();

// Convenience helpers that safely query the active driver and return
// fallbacks when no driver is installed. Use these to avoid repetitive
// null-checks at call-sites.
int getActiveMotorId(IMotorDriver::MotorSide side, int fallback = -1);
bool isActiveMotorInverted(IMotorDriver::MotorSide side, bool fallback = false);
float getActiveVelocityMaxSpeed(float fallback = 0.0f);
float getActiveVelocityTargetIncrementScale(float fallback = 0.0f);
float getActiveVelocityPositionKp(float fallback = 0.0f);
const char *getActiveDriverName(const char *fallback = "none");

// Convenience: install the default servo-backed adapter (delegates to the
// existing servo functions). Call this at startup if you want the current
// servo code to act as the driver for the manager.
void installDefaultServoAdapter();

// Install the default motor driver selected by configuration.
// This abstracts which concrete driver is used so callers (e.g. `main.cpp`)
// only depend on the manager API and not on concrete driver headers.
void installDefaultMotorDriver();

// --- Compatibility wrappers (ID-based) ---
// These helpers ease migration for code that still references numeric motor
// IDs. They map numeric IDs to the configured `MotorSide` via the active
// driver's `getMotorId()` and forward to the `IMotorDriver` methods.
void setMotorCommandById(int id, float command);
void setMotorCommandRawById(int id, int16_t rawSpeed);
int32_t readEncoderById(int id);

/**
 * Encoder report returned by `getEncoderReportFromArg`.
 *
 * All integer fields use fixed-width types for portability.
 */
struct EncoderReport {
	bool ok;           ///< true when parsing succeeded and report populated
	bool both;         ///< true when report contains both encoders
	int requestedId;   ///< numeric id requested, or -1 when both
	int leftId;        ///< configured left motor id, -1 if none
	int rightId;       ///< configured right motor id, -1 if none
	int32_t leftVal;   ///< left encoder value (valid for both or left)
	int32_t rightVal;  ///< right encoder value (valid for both or right)
};

/**
 * Parse textual argument and populate `out`.
 *
 * Accepts C-style string `arg`. A nullptr or empty string means "both".
 * Returns true on successful parse/report; returns false when the token
 * is syntactically invalid (caller should print usage).
 *
 * Thread-safety: this function is safe to call from the Wi‑Fi/serial task
 * and other contexts because it acquires internal protection while
 * reading the active driver pointer. Internally it copies the active driver
 * pointer under a short critical-section and then invokes driver methods
 * outside the critical-section to avoid nested locking or long-held locks.
 *
 * Locking notes for other helpers:
 * - `getActiveMotorDriver()` returns the active pointer and uses an
 *   internal mutex for atomic reads. Callers should not assume the driver
 *   remains valid after return unless they take ownership or synchronize
 *   externally.
 * - Convenience helpers such as `getActiveMotorId()` call into the active
 *   driver via the manager and are safe to use from task contexts; they
 *   perform short internal locking when required.
 */
bool getEncoderReportFromArg(const char *arg, EncoderReport &out);

// No free-function forwards here; callers should query `getActiveMotorDriver()`
// and call the appropriate `IMotorDriver` methods, or use the small
// `motor_control` wrapper for high-level enable/disable queries.

} // namespace motor
} // namespace abbot
