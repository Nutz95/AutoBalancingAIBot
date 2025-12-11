// driver_manager.h
#pragma once
#include "IMotorDriver.h"

namespace abbot {
namespace motor {

// Set the active motor driver implementation. Ownership remains with caller.
void setActiveMotorDriver(IMotorDriver *drv);

// Get the currently active driver (may be nullptr if none set).
IMotorDriver *getActiveMotorDriver();

// Convenience: install the default servo-backed adapter (delegates to the
// existing servo functions). Call this at startup if you want the current
// servo code to act as the driver for the manager.
void installDefaultServoAdapter();

// No free-function forwards here; callers should query `getActiveMotorDriver()`
// and call the appropriate `IMotorDriver` methods, or use the small
// `motor_control` wrapper for high-level enable/disable queries.

} // namespace motor
} // namespace abbot
