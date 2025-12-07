// IMU filter interface: abstract wrapper for Madgwick, Complementary 1D, Kalman 1D
#pragma once
#include "imu_fusion.h"

namespace abbot {

class IMUFilter {
public:
  virtual ~IMUFilter() {}
  // Initialize filter with FusionConfig (contains sample_rate and axis mapping)
  virtual void begin(const fusion::FusionConfig &cfg) = 0;
  // Update filter with gyro (rad/s) and accel (m/s^2) in robot axes
  virtual void update(float gx, float gy, float gz, float ax, float ay, float az, float dt) = 0;
  // Optional: seed orientation from accel averages
  virtual void setFromAccel(float ax, float ay, float az) { (void)ax; (void)ay; (void)az; }
  // Query angle/rate
  virtual float getPitch() = 0;
  virtual float getPitchRate() = 0;
  // How long (ms) should a warmup be requested after switching to this filter.
  // Default 0 = no special warmup requested.
  virtual unsigned long getWarmupDurationMs() const { return 0; }
  // Optional: return quaternion if available
  virtual void getQuaternion(float &w, float &x, float &y, float &z) { w = x = y = z = 0.0f; }
  // Runtime parameter API: allow filters to expose named float parameters
  // (e.g. complementary filter 'ALPHA'). Default implementations return false
  // to indicate parameter not supported.
  virtual bool setParam(const char *name, float value) { (void)name; (void)value; return false; }
  virtual bool getParam(const char *name, float &out) { (void)name; (void)out; return false; }
};

} // namespace abbot
