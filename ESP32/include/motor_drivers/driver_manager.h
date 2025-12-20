// driver_manager.h
#pragma once
#include "IMotorDriver.h"

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

// No free-function forwards here; callers should query `getActiveMotorDriver()`
// and call the appropriate `IMotorDriver` methods, or use the small
// `motor_control` wrapper for high-level enable/disable queries.

} // namespace motor
} // namespace abbot
