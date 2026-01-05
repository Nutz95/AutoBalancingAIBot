// imu_manager.h
#pragma once

#include "IIMUDriver.h"

namespace abbot {
namespace imu {

/**
 * @brief Set the active IMU driver.
 * @param drv Pointer to the driver instance.
 */
void setActiveIMUDriver(IIMUDriver *drv);

/**
 * @brief Get the currently active IMU driver.
 * @return Pointer to the active driver, or nullptr if none.
 */
IIMUDriver *getActiveIMUDriver();

/**
 * @brief Helper to get the name of the active driver.
 * @param fallback String to return if no driver is active.
 * @return The driver name or fallback.
 */
const char *getActiveDriverName(const char *fallback = "none");

} // namespace imu
} // namespace abbot
