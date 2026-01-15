// BMI160Driver.h
#pragma once

#include "../../config/imu_configs/BMI160Config.h"
#include "IIMUDriver.h"
#include <Arduino.h>
#include <BMI160Gen.h>
#include <Wire.h>

namespace abbot {

/**
 * @brief Driver for the BMI160 IMU using I2C.
 * 
 * This driver uses the BMI160Gen library.
 * It provides raw accelerometer and gyroscope data at high frequency.
 */
class BMI160Driver : public IIMUDriver {
public:
  explicit BMI160Driver(const BMI160Config &cfg);
  virtual ~BMI160Driver() {
  }

  bool begin() override;
  bool read(IMUSample &out) override;
  bool readRaw(IMUSample &out) override;

  uint16_t getSamplingHz() const override;

  const char* getDriverName() const override;

  int8_t getPitchRateSign() const override {
    return cfg_.pitch_rate_sign;
  }

  bool isCalibrated() const override {
      return false; // BMI160 requires external calibration
  }

private:
  BMI160Config cfg_;
  unsigned long last_read_us_ = 0;
  bool has_dummy_ = true;
  
  // Internal helper to map gyro/accel signs
  void applySigns(IMUSample &out);
};

} // namespace abbot
