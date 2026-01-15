// BNO055Driver.h
#pragma once

#include "../../config/imu_configs/BNO055Config.h"
#include "IIMUDriver.h"
#include <Adafruit_BNO055.h>
#include <Arduino.h>
#include <Wire.h>

namespace abbot {

/**
 * @brief Driver for the BNO055 IMU using I2C.
 * 
 * This driver uses the Adafruit BNO055 library. It supports reading
 * raw data or fused data. For compatibility with the existing fusion
 * service, it provides raw accelerometer and gyroscope data.
 */
class BNO055Driver : public IIMUDriver {
public:
  explicit BNO055Driver(const BNO055Config &cfg);
  virtual ~BNO055Driver() {
  }

  bool begin() override;
  bool read(IMUSample &out) override;
  bool readRaw(IMUSample &out) override;

  uint16_t getSamplingHz() const override;

  const char* getDriverName() const override;

  int8_t getPitchRateSign() const override {
    return cfg_.pitch_rate_sign;
  }

  bool isCalibrated() const override;

private:
  BNO055Config cfg_;
  Adafruit_BNO055 bno_;
  unsigned long last_read_us_ = 0;
};

} // namespace abbot
