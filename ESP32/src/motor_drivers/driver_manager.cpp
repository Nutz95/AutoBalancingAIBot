// driver_manager.cpp
#include "../include/motor_drivers/driver_manager.h"
#include "../config/motor_configs/motor_common_config.h"
#include "../include/motor_drivers/IMotorDriver.h"
#include "logging.h"
#if MOTOR_USE_DC_DRIVER
#include "../include/motor_drivers/dc_mirror_driver.h"
#endif
#if MOTOR_USE_MKS_SERVO
#include "../include/motor_drivers/MksServoMotorDriver.h"
#endif
#include <cstdlib>
#include <cctype>
#include <cstring>
#if !defined(UNIT_TEST_HOST)
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/portmacro.h>
#include <esp_attr.h>
#else
// Provide minimal host-side stub for portMUX_TYPE used in unit tests
typedef int portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED 0
#endif

namespace abbot {
namespace motor {

static IMotorDriver *g_active = nullptr;
static portMUX_TYPE g_active_mux = portMUX_INITIALIZER_UNLOCKED;

// Use shared RAII guard for critical sections
#include "../include/motor_drivers/critical_guard.h"
using abbot::motor::CriticalGuard;

void setActiveMotorDriver(IMotorDriver *drv) {
  // Protect assignment to g_active to avoid races with concurrent readers
  {
    CriticalGuard g(&g_active_mux);
    g_active = drv;
  }
  LOG_PRINTLN(::abbot::log::CHANNEL_MOTOR,
              "driver_manager: active motor driver set");
}

IMotorDriver *getActiveMotorDriver() {
  IMotorDriver *tmp = nullptr;
  {
    CriticalGuard g(&g_active_mux);
    tmp = g_active;
  }
  return tmp;
}

// installDefaultServoAdapter is implemented in servo_adapter.cpp
void installDefaultServoAdapter();

#if MOTOR_USE_MKS_SERVO
static MksServoMotorDriver g_mksServoDriver;
void installDefaultMksServoDriver() { setActiveMotorDriver(&g_mksServoDriver); }
#endif

// No free-function forwards implemented here. Callers should query
// `getActiveMotorDriver()` and invoke methods on the returned driver.

// Auto-install the servo adapter by default so existing code that wants
// to use the manager can call `installDefaultServoAdapter()` during init
// or rely on this implicit install below by calling it explicitly in setup.

// --- Helper implementations ---
int getActiveMotorId(IMotorDriver::MotorSide side, int fallback) {
  if (auto drv = getActiveMotorDriver()) {
    return drv->getMotorId(side);
  }
  return fallback;
}

bool isActiveMotorInverted(IMotorDriver::MotorSide side, bool fallback) {
  if (auto drv = getActiveMotorDriver()) {
    return drv->isMotorInverted(side);
  }
  return fallback;
}

float getActiveVelocityMaxSpeed(float fallback) {
  if (auto drv = getActiveMotorDriver()) {
    return drv->getVelocityMaxSpeed();
  }
  return fallback;
}

float getActiveVelocityTargetIncrementScale(float fallback) {
  if (auto drv = getActiveMotorDriver()) {
    return drv->getVelocityTargetIncrementScale();
  }
  return fallback;
}

float getActiveVelocityPositionKp(float fallback) {
  if (auto drv = getActiveMotorDriver()) {
    return drv->getVelocityPositionKp();
  }
  return fallback;
}

const char *getActiveDriverName(const char *fallback) {
  if (auto drv = getActiveMotorDriver()) {
    return drv->getDriverName();
  }
  return fallback;
}

// Install the default motor driver according to compile-time config.
void installDefaultMotorDriver() {
#if MOTOR_USE_MKS_SERVO
  installDefaultMksServoDriver();
#elif MOTOR_USE_DC_DRIVER
  installDefaultDCMirrorDriver();
#else
  installDefaultServoAdapter();
#endif
}

// Helper: map numeric id -> MotorSide using active driver's getMotorId
// Note: mapping id->side is provided by `getSideForId`; avoid duplicate
// static helpers to keep the file simpler and satisfy verifier checks.

void setMotorCommandBySide(IMotorDriver::MotorSide side, float command) {
  IMotorDriver *drv = nullptr;
  {
    CriticalGuard g(&g_active_mux);
    drv = g_active;
  }
  if (!drv)
    return;
  drv->setMotorCommand(side, command);
}

void setMotorCommandRawBySide(IMotorDriver::MotorSide side, int16_t rawSpeed) {
  IMotorDriver *drv = nullptr;
  {
    CriticalGuard g(&g_active_mux);
    drv = g_active;
  }
  if (!drv)
    return;
  drv->setMotorCommandRaw(side, rawSpeed);
}

int32_t readEncoderBySide(IMotorDriver::MotorSide side) {
  IMotorDriver *drv = nullptr;
  {
    CriticalGuard g(&g_active_mux);
    drv = g_active;
  }
  if (!drv)
    return 0;
  return drv->readEncoder(side);
}

float readSpeedBySide(IMotorDriver::MotorSide side) {
  IMotorDriver *drv = nullptr;
  {
    CriticalGuard g(&g_active_mux);
    drv = g_active;
  }
  if (!drv)
    return 0.0f;
  return drv->readSpeed(side);
}

bool getSideForId(int id, IMotorDriver::MotorSide &out) {
  IMotorDriver *drv = nullptr;
  {
    CriticalGuard g(&g_active_mux);
    drv = g_active;
  }
  if (!drv)
    return false;
  IMotorDriver::MotorSide sides[2] = {IMotorDriver::MotorSide::LEFT,
                                      IMotorDriver::MotorSide::RIGHT};
  for (int i = 0; i < 2; ++i) {
    IMotorDriver::MotorSide s = sides[i];
    if (drv->getMotorId(s) == id) {
      out = s;
      return true;
    }
  }
  return false;
}

// Protect access to g_active and encoder reads
// (mutex declared above near the top of the file)

bool getEncoderReportFromArg(const char *arg, EncoderReport &out) {
  // Initialize
  out.ok = false;
  out.both = false;
  out.requestedId = -1;
  out.leftId = -1;
  out.rightId = -1;
  out.leftVal = 0;
  out.rightVal = 0;

  // Interpret nullptr or empty as "both"
  if (!arg || arg[0] == '\0') {
    // Copy active driver pointer under lock, then call methods on local
    // driver pointer outside the critical section to avoid nested locking.
    IMotorDriver *drv = nullptr;
    {
      CriticalGuard g(&g_active_mux);
      drv = g_active;
    }

    if (!drv) {
      out.leftId = -1;
      out.rightId = -1;
      out.leftVal = 0;
      out.rightVal = 0;
      out.both = true;
      out.ok = true;
      return true;
    }

    out.leftId = drv->getMotorId(IMotorDriver::MotorSide::LEFT);
    out.rightId = drv->getMotorId(IMotorDriver::MotorSide::RIGHT);
    out.leftVal = drv->readEncoder(IMotorDriver::MotorSide::LEFT);
    out.rightVal = drv->readEncoder(IMotorDriver::MotorSide::RIGHT);
    out.both = true;
    out.ok = true;
    return true;
  }

  // Token handling: allow LEFT/RIGHT (case-insensitive) or numeric id
  // Avoid Arduino `String` to prevent heap fragmentation; use small stack buffer.
  char tokbuf[32];
  size_t srcLen = strlen(arg);
  if (srcLen >= sizeof(tokbuf))
    srcLen = sizeof(tokbuf) - 1;
  memcpy(tokbuf, arg, srcLen);
  tokbuf[srcLen] = '\0';
  // Trim leading/trailing whitespace in-place
  char *start = tokbuf;
  while (*start && isspace((unsigned char)*start))
    ++start;
  char *end = start + strlen(start);
  while (end > start && isspace((unsigned char)*(end - 1)))
    *--end = '\0';
  if (strcasecmp(start, "LEFT") == 0) {
    IMotorDriver *drv = nullptr;
    {
      CriticalGuard g(&g_active_mux);
      drv = g_active;
    }
    out.requestedId = -1;
    if (!drv) {
      out.leftVal = 0;
      out.ok = true;
      return true;
    }
    out.requestedId = drv->getMotorId(IMotorDriver::MotorSide::LEFT);
    out.leftVal = drv->readEncoder(IMotorDriver::MotorSide::LEFT);
    out.ok = true;
    return true;
  } else if (strcasecmp(start, "RIGHT") == 0) {
    IMotorDriver *drv = nullptr;
    {
      CriticalGuard g(&g_active_mux);
      drv = g_active;
    }
    out.requestedId = -1;
    if (!drv) {
      out.rightVal = 0;
      out.ok = true;
      return true;
    }
    out.requestedId = drv->getMotorId(IMotorDriver::MotorSide::RIGHT);
    out.rightVal = drv->readEncoder(IMotorDriver::MotorSide::RIGHT);
    out.ok = true;
    return true;
  }

  // Numeric parse with validation
  const char *c = start;
  char *endptr = nullptr;
  long v = strtol(c, &endptr, 10);
  if (endptr == c) {
    // parse failed
    return false;
  }
  // range check
  if (v < INT32_MIN || v > INT32_MAX) {
    return false;
  }
  out.requestedId = (int)v;
  IMotorDriver *drv = nullptr;
  {
    CriticalGuard g(&g_active_mux);
    drv = g_active;
  }

  if (!drv) {
    out.leftVal = 0;
    out.rightVal = 0;
    out.ok = true;
    return true;
  }

  // Map numeric id to side by asking the driver for its configured ids.
  int leftId = drv->getMotorId(IMotorDriver::MotorSide::LEFT);
  int rightId = drv->getMotorId(IMotorDriver::MotorSide::RIGHT);
  if (out.requestedId == leftId) {
    out.leftVal = drv->readEncoder(IMotorDriver::MotorSide::LEFT);
    out.rightVal = out.leftVal;
  } else if (out.requestedId == rightId) {
    out.rightVal = drv->readEncoder(IMotorDriver::MotorSide::RIGHT);
    out.leftVal = out.rightVal;
  } else {
        LOG_PRINTF(::abbot::log::CHANNEL_MOTOR,
          "driver_manager: readEncoderById: unknown numeric id=%d", out.requestedId);
    out.leftVal = 0;
    out.rightVal = 0;
  }
  out.ok = true;
  return true;
}

} // namespace motor
} // namespace abbot
