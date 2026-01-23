#include <cmath>
#include <cstring>
#include <esp_attr.h>

#include "../../config/imu_filter_config.h"
#include "imu_filter.h"

namespace abbot {

class ComplementaryFilter1D : public IMUFilter {
public:
  ComplementaryFilter1D()
      : k_acc_(0.02f), k_bias_(0.01f), angle_(0.0f), rate_(0.0f),
        bias_(0.0f) {
  }
  void begin(const fusion::FusionConfig &cfg) override {
    (void)cfg;
    reset();
  }
  void reset() override {
    angle_ = 0.0f;
    rate_ = 0.0f;
    bias_ = 0.0f;
  }
  unsigned long getWarmupDurationMs() const override {
    return IMU_FILTER_WARMUP_MS_COMPLEMENTARY1D;
  }
  void update(float gx, float gy, float gz, float ax, float ay, float az,
              float dt, float fused_pitch = 0.0f,
              float fused_roll = 0.0f, float fused_yaw = 0.0f) override {
    (void)fused_pitch;
    (void)fused_roll;
    (void)fused_yaw;
    // Compute accel pitch (approx): assuming robot x axis forward
    // Compute pitch using robot-frame X axis (positive when tilting forward)
    float accel_pitch = atan2f(ax, sqrtf(ay * ay + az * az));
    // integrate gyro (assume gyro y is pitch rate for rotation in the X-Z plane)
    float gyro_rate = gy; // Corrected sign for v46 (Forward = Positive)

    // Estimate gyro bias using accel-correction error (simple PI-like form)
    float error = accel_pitch - angle_;
    bias_ += k_bias_ * error * dt;

    float gyro_unbiased = gyro_rate - bias_;
    angle_ += gyro_unbiased * dt;
    // Complementary accel correction
    angle_ += k_acc_ * error;
    rate_ = gyro_unbiased;
  }
  void setFromAccel(float ax, float ay, float az) override {
    float accel_pitch = atan2f(ax, sqrtf(ay * ay + az * az));
    angle_ = accel_pitch;
    rate_ = 0.0f;
    bias_ = 0.0f;
  }
  float getPitch() override {
    return angle_;
  }
  float getPitchRate() override {
    return rate_;
  }
  // Parameter API: support KACC (accel correction) and KBIAS (bias integrator)
  bool setParam(const char *name, float value) override {
    if (!name) {
      return false;
    }
    if (strcmp(name, "KACC") == 0) {
      if (value <= 0.0f) {
        return false;
      }
      if (value > 1.0f) {
        value = 1.0f;
      }
      k_acc_ = value;
      return true;
    }
    if (strcmp(name, "KBIAS") == 0) {
      if (value < 0.0f) {
        return false;
      }
      if (value > 1.0f) {
        value = 1.0f;
      }
      k_bias_ = value;
      return true;
    }
    return false;
  }
  bool getParam(const char *name, float &out) override {
    if (!name) {
      return false;
    }
    if (strcmp(name, "KACC") == 0) {
      out = k_acc_;
      return true;
    }
    if (strcmp(name, "KBIAS") == 0) {
      out = k_bias_;
      return true;
    }
    return false;
  }

private:
  float k_acc_;
  float k_bias_;
  float angle_;
  float rate_;
  float bias_;
};

IMUFilter *createComplementaryFilter1D() {
  return new ComplementaryFilter1D();
}

} // namespace abbot
