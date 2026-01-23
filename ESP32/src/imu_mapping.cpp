#include "imu_mapping.h"
#include <esp_attr.h>

namespace abbot {
namespace imu_mapping {

void IRAM_ATTR mapSensorToRobot(const fusion::FusionConfig &cfg, const float raw_g[3],
                      const float raw_a[3], const float gyro_bias[3],
                      float out_g[3], float out_a[3]) {
  float sensor_g[3] = {raw_g[0] - gyro_bias[0], raw_g[1] - gyro_bias[1],
                       raw_g[2] - gyro_bias[2]};
  float sensor_a[3] = {raw_a[0], raw_a[1], raw_a[2]};
  for (int i = 0; i < 3; ++i) {
    int gim = cfg.gyro_map[i];
    int gsign = cfg.gyro_sign[i];
    int aim = cfg.accel_map[i];
    int asign = cfg.accel_sign[i];
    if (gim < 0 || gim > 2)
      gim = i;
    if (aim < 0 || aim > 2)
      aim = i;
    out_g[i] = (float)gsign * sensor_g[gim];
    out_a[i] = (float)asign * sensor_a[aim];
  }
}

} // namespace imu_mapping
} // namespace abbot
