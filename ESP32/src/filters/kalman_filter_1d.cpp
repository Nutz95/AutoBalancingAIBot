#include "../../config/imu_filter_config.h"
#include "imu_filter.h"
#include <cmath>

namespace abbot {

// Simple 1D Kalman filter estimating angle and gyro bias.
class KalmanFilter1D : public IMUFilter {
public:
  KalmanFilter1D() {
    angle_ = 0.0f;
    bias_ = 0.0f;
    P00 = 1e-3f;
    P01 = 0.0f;
    P10 = 0.0f;
    P11 = 1e-3f;
    Q_angle = 1e-4f;
    Q_bias = 1e-5f;
    R_measure = 0.03f;
    rate_ = 0.0f;
  }
  void begin(const fusion::FusionConfig &cfg) override { (void)cfg; }
  unsigned long getWarmupDurationMs() const override {
    return IMU_FILTER_WARMUP_MS_KALMAN1D;
  }
  void update(float gx, float gy, float gz, float ax, float ay, float az,
              float dt) override {
    // Predict
    float gyro_rate = gx - bias_;
    angle_ += gyro_rate * dt;
    // Update error covariance
    P00 += dt * (dt * P11 - P01 - P10 + Q_angle);
    P01 -= dt * P11;
    P10 -= dt * P11;
    P11 += Q_bias * dt;

    // Measurement
    float accel_angle = atan2f(-ax, sqrtf(ay * ay + az * az));
    float y = accel_angle - angle_;
    float S = P00 + R_measure;
    float K0 = P00 / S;
    float K1 = P10 / S;

    // Update state
    angle_ += K0 * y;
    bias_ += K1 * y;

    // Update covariance
    float P00_tmp = P00;
    float P01_tmp = P01;
    P00 -= K0 * P00_tmp;
    P01 -= K0 * P01_tmp;
    P10 -= K1 * P00_tmp;
    P11 -= K1 * P01_tmp;

    rate_ = gyro_rate + bias_; // approximate
  }
  float getPitch() override { return angle_; }
  float getPitchRate() override { return rate_; }

private:
  float angle_, bias_, rate_;
  float P00, P01, P10, P11;
  float Q_angle, Q_bias, R_measure;
};

IMUFilter *createKalmanFilter1D() { return new KalmanFilter1D(); }

} // namespace abbot
