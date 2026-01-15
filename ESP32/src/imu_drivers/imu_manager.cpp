// imu_manager.cpp
#include "../../include/imu_drivers/imu_manager.h"
#include "../../include/logging.h"

#if !defined(UNIT_TEST_HOST)
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/portmacro.h>
#include <esp_attr.h>
#else
typedef int portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED 0
#define portENTER_CRITICAL(mux)
#define portEXIT_CRITICAL(mux)
#endif

namespace abbot {
namespace imu {

static IIMUDriver *g_active_imu = nullptr;
static portMUX_TYPE g_imu_mux = portMUX_INITIALIZER_UNLOCKED;

void setActiveIMUDriver(IIMUDriver *drv) {
#if !defined(UNIT_TEST_HOST)
  portENTER_CRITICAL(&g_imu_mux);
#endif
  g_active_imu = drv;
#if !defined(UNIT_TEST_HOST)
  portEXIT_CRITICAL(&g_imu_mux);
#endif
  
  if (drv) {
    LOG_PRINTF(abbot::log::CHANNEL_DEFAULT, "imu_manager: active IMU set to %s\n", drv->getDriverName());
  } else {
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "imu_manager: active IMU cleared");
  }
}

IIMUDriver *getActiveIMUDriver() {
  IIMUDriver *active_driver = nullptr;
#if !defined(UNIT_TEST_HOST)
  portENTER_CRITICAL(&g_imu_mux);
#endif
  active_driver = g_active_imu;
#if !defined(UNIT_TEST_HOST)
  portEXIT_CRITICAL(&g_imu_mux);
#endif
  return active_driver;
}

const char *getActiveDriverName(const char *fallback) {
  IIMUDriver *drv = getActiveIMUDriver();
  if (drv) {
    return drv->getDriverName();
  }
  return fallback;
}

int8_t getActivePitchRateSign() {
  IIMUDriver *drv = getActiveIMUDriver();
  if (drv) {
    return drv->getPitchRateSign();
  }
  return 1;
}

} // namespace imu
} // namespace abbot
