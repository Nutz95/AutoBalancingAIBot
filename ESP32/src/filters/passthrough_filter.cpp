#include "../../include/imu_filter.h"
#include <cstring>

namespace abbot {

/**
 * @brief A "Passthrough" filter that uses the sensor's internal fusion data.
 * 
 * This filter does not perform any integration or fusion itself. It simply
 * returns the fused_pitch and fused_roll values provided by the sensor driver
 * (e.g., BNO055).
 */
class PassthroughFilter : public IMUFilter {
public:
  PassthroughFilter()
      : pitch_(0.0f), roll_(0.0f), yaw_(0.0f), rate_(0.0f) {
  }
  virtual ~PassthroughFilter() {
  }

  void begin(const fusion::FusionConfig &cfg) override {
    (void)cfg;
    reset();
  }

  void reset() override {
    pitch_ = 0.0f;
    roll_ = 0.0f;
    yaw_ = 0.0f;
    rate_ = 0.0f;
  }

  void update(float gx, float gy, float gz, float ax, float ay, float az,
              float dt, float fused_pitch = 0.0f,
              float fused_roll = 0.0f, float fused_yaw = 0.0f) override {
    (void)gy;
    (void)gz;
    (void)ax;
    (void)ay;
    (void)az;
    (void)dt;

    // Use the fused data directly from the sensor
    pitch_ = fused_pitch;
    roll_ = fused_roll;
    yaw_ = fused_yaw;
    // Use the raw gyro rate (robot frame) for the D-term of the PID
    rate_ = gx;
  }

  float getPitch() override {
    return pitch_;
  }
  float getPitchRate() override {
    return rate_;
  }
  float getYaw() override {
    return yaw_;
  }
  float getRoll() override {
    return roll_;
  }

private:
  float pitch_;
  float roll_;
  float yaw_;
  float rate_;
};

// Factory helper
IMUFilter *createPassthroughFilter() {
  return new PassthroughFilter();
}

} // namespace abbot
