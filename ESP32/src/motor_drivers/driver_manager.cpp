// driver_manager.cpp
#include "../include/motor_drivers/driver_manager.h"
#include "../include/motor_drivers/IMotorDriver.h"
#include "logging.h"

namespace abbot {
namespace motor {

static IMotorDriver *g_active = nullptr;

void setActiveMotorDriver(IMotorDriver *drv) {
  g_active = drv;
  LOG_PRINTLN(abbot::log::CHANNEL_MOTOR, "driver_manager: active motor driver set");
}

IMotorDriver *getActiveMotorDriver() {
  return g_active;
}

// installDefaultServoAdapter is implemented in servo_adapter.cpp
void installDefaultServoAdapter();

// No free-function forwards implemented here. Callers should query
// `getActiveMotorDriver()` and invoke methods on the returned driver.

// Auto-install the servo adapter by default so existing code that wants
// to use the manager can call `installDefaultServoAdapter()` during init
// or rely on this implicit install below by calling it explicitly in setup.

} // namespace motor
} // namespace abbot
