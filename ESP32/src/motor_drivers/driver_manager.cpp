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

// --- Helper implementations ---
int getActiveMotorId(IMotorDriver::MotorSide side, int fallback) {
  if (g_active) {
    return g_active->getMotorId(side);
  }
  return fallback;
}

bool isActiveMotorInverted(IMotorDriver::MotorSide side, bool fallback) {
  if (g_active) {
    return g_active->isMotorInverted(side);
  }
  return fallback;
}

float getActiveVelocityMaxSpeed(float fallback) {
  if (g_active) {
    return g_active->getVelocityMaxSpeed();
  }
  return fallback;
}

float getActiveVelocityTargetIncrementScale(float fallback) {
  if (g_active) {
    return g_active->getVelocityTargetIncrementScale();
  }
  return fallback;
}

float getActiveVelocityPositionKp(float fallback) {
  if (g_active) {
    return g_active->getVelocityPositionKp();
  }
  return fallback;
}

const char* getActiveDriverName(const char* fallback) {
  if (g_active) {
    return g_active->getDriverName();
  }
  return fallback;
}

// Helper: map numeric id -> MotorSide using active driver's getMotorId
static bool idToSide(int id, IMotorDriver::MotorSide &out_side) {
  if (!g_active) return false;
  IMotorDriver::MotorSide sides[2] = { IMotorDriver::MotorSide::LEFT, IMotorDriver::MotorSide::RIGHT };
  for (int i = 0; i < 2; ++i) {
    IMotorDriver::MotorSide s = sides[i];
    if (g_active->getMotorId(s) == id) {
      out_side = s;
      return true;
    }
  }
  return false;
}

void setMotorCommandById(int id, float command) {
  if (!g_active) return;
  IMotorDriver::MotorSide side;
  if (idToSide(id, side)) {
    g_active->setMotorCommand(side, command);
  } else {
    LOG_PRINTF(abbot::log::CHANNEL_MOTOR, "driver_manager: setMotorCommandById unknown id=%d", id);
  }
}

void setMotorCommandRawById(int id, int16_t rawSpeed) {
  if (!g_active) return;
  IMotorDriver::MotorSide side;
  if (idToSide(id, side)) {
    g_active->setMotorCommandRaw(side, rawSpeed);
  } else {
    LOG_PRINTF(abbot::log::CHANNEL_MOTOR, "driver_manager: setMotorCommandRawById unknown id=%d", id);
  }
}

int32_t readEncoderById(int id) {
  if (!g_active) return 0;
  IMotorDriver::MotorSide side;
  if (idToSide(id, side)) {
    return g_active->readEncoder(side);
  }
  LOG_PRINTF(abbot::log::CHANNEL_MOTOR, "driver_manager: readEncoderById unknown id=%d", id);
  return 0;
}

} // namespace motor
} // namespace abbot
