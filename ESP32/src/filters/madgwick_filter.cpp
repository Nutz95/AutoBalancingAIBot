#include "../../config/imu_filter_config.h"
#include "imu_filter.h"
#include "imu_fusion.h"
#include <cstring>

namespace abbot {

class MadgwickFilter : public IMUFilter {
public:
  MadgwickFilter() {}
  virtual ~MadgwickFilter() {}
  void begin(const fusion::FusionConfig &cfg) override { madgwick_.begin(cfg); }
  void reset() override { madgwick_.reset(); }
  unsigned long getWarmupDurationMs() const override {
    return IMU_FILTER_WARMUP_MS_MADGWICK;
  }
  void update(float gx, float gy, float gz, float ax, float ay, float az,
              float dt) override {
    madgwick_.update(gx, gy, gz, ax, ay, az, dt);
  }
  void setFromAccel(float ax, float ay, float az) override {
    madgwick_.setFromAccel(ax, ay, az);
  }
  float getPitch() override { return madgwick_.getPitch(); }
  float getPitchRate() override { return madgwick_.getPitchRate(); }
  void getQuaternion(float &w, float &x, float &y, float &z) override {
    madgwick_.getQuaternion(w, x, y, z);
  }

  bool setParam(const char *name, float value) override {
    if (!name) return false;
    if (strcmp(name, "BETA") == 0) {
      madgwick_.setBeta(value);
      return true;
    }
    return false;
  }

  bool getParam(const char *name, float &out) override {
    if (!name) return false;
    if (strcmp(name, "BETA") == 0) {
      out = madgwick_.getBeta();
      return true;
    }
    return false;
  }

private:
  fusion::Madgwick madgwick_;
};

// Factory helper
IMUFilter *createMadgwickFilter() { return new MadgwickFilter(); }

} // namespace abbot
