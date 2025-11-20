// BMI088Driver.cpp
#include "BMI088Driver.h"

namespace abbot {

BMI088Driver::BMI088Driver(const BMI088Config &cfg) : cfg_(cfg), imu_(SPI, cfg.accel_cs_pin, cfg.gyro_cs_pin) {
}

bool BMI088Driver::begin() {
  // Initialize SPI and sensor
  SPI.begin();
  int rc = imu_.begin();
  if (rc != 0) {
    return false;
  }
  // Try to set ODR to 400Hz on sensor if available, we'll sample at 200Hz
  imu_.setOdr(Bmi088::ODR_400HZ);
  last_read_ms_ = millis();
  return true;
}

bool BMI088Driver::read(IMUSample &out) {
  unsigned long now = millis();
  unsigned long interval = cfg_.sampling_interval_ms();
  if (interval == 0) return false;
  if ((now - last_read_ms_) < interval) return false; // not time yet
  imu_.readSensor();
  out.ax = imu_.getAccelX_mss();
  out.ay = imu_.getAccelY_mss();
  out.az = imu_.getAccelZ_mss();
  out.gx = imu_.getGyroX_rads();
  out.gy = imu_.getGyroY_rads();
  out.gz = imu_.getGyroZ_rads();
  out.time_ps = imu_.getTime_ps();
  out.ts_ms = now;
  last_read_ms_ = now;
  return true;
}

} // namespace abbot
