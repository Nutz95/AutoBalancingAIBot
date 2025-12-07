#include "imu_filter.h"
#include "../../config/imu_filter_config.h"
#include <cmath>
#include <cstring>

namespace abbot {

class ComplementaryFilter1D : public IMUFilter {
public:
  ComplementaryFilter1D() : alpha_(0.98f), angle_(0.0f), rate_(0.0f) {}
  void begin(const fusion::FusionConfig &cfg) override {
    (void)cfg;
    angle_ = 0.0f;
    rate_ = 0.0f;
  }
  unsigned long getWarmupDurationMs() const override { return IMU_FILTER_WARMUP_MS_COMPLEMENTARY1D; }
  void update(float gx, float gy, float gz, float ax, float ay, float az, float dt) override {
    // Compute accel pitch (approx): assuming robot x axis forward
    float accel_pitch = atan2f(-ax, sqrtf(ay*ay + az*az));
    // integrate gyro (assume gyro x is pitch rate)
    float gyro_rate = gx; // rad/s
    angle_ = alpha_ * (angle_ + gyro_rate * dt) + (1.0f - alpha_) * accel_pitch;
    rate_ = gyro_rate;
  }
  float getPitch() override { return angle_; }
  float getPitchRate() override { return rate_; }
  // Parameter API: support ALPHA tuning at runtime
  bool setParam(const char *name, float value) override {
    if (!name) return false;
    // tokens passed from serial command are upper-case; compare directly
    if (strcmp(name, "ALPHA") == 0) {
      // constrain plausible range
      if (value <= 0.0f) return false;
      if (value > 1.0f) value = 1.0f;
      alpha_ = value;
      return true;
    }
    return false;
  }
  bool getParam(const char *name, float &out) override {
    if (!name) return false;
    if (strcmp(name, "ALPHA") == 0) {
      out = alpha_;
      return true;
    }
    return false;
  }
private:
  float alpha_;
  float angle_;
  float rate_;
};

IMUFilter* createComplementaryFilter1D() { return new ComplementaryFilter1D(); }

} // namespace abbot
