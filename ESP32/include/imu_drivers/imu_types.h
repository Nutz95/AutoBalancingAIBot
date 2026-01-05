// imu_types.h
#pragma once

#include <stdint.h>

namespace abbot {

/**
 * @brief Standard IMU sample structure used across the project.
 * 
 * This structure holds raw or calibrated sensor data and associated timestamps.
 */
struct IMUSample {
  float ax, ay, az;              // m/s^2
  float gx, gy, gz;              // rad/s
  float temperatureCelsius;      // sensor temperature Celsius
  float fused_pitch;             // rad (internal fusion if available)
  float fused_roll;              // rad (internal fusion if available)
  float fused_yaw;               // rad (internal fusion if available, e.g. Heading)
  uint64_t time_ps;              // sensor timestamp in picoseconds (if available)
  unsigned long ts_ms;           // host timestamp (millis)
  unsigned long ts_us;           // host timestamp (micros)
};

} // namespace abbot
