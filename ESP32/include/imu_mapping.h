#pragma once
#include "imu_fusion.h"

namespace abbot {
namespace imu_mapping {
// Map sensor-frame gyro/accel into robot-frame using FusionConfig and a
// runtime gyro-bias. Arrays are 3-element floats: [x,y,z].
void mapSensorToRobot(const fusion::FusionConfig &cfg, const float raw_g[3],
                      const float raw_a[3], const float gyro_bias[3],
                      float out_g[3], float out_a[3]);

} // namespace imu_mapping
} // namespace abbot
