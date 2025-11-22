// BMI088Driver.cpp
#include "BMI088Driver.h"
#include "imu_calibration.h"

namespace abbot {

BMI088Driver::BMI088Driver(const BMI088Config &cfg) : cfg_(cfg), imu_(SPI, cfg.accel_cs_pin, cfg.gyro_cs_pin) {
}

bool BMI088Driver::begin() {
  // Initialize SPI and sensor
  SPI.begin();
  Serial.print("BMI088: accel_cs="); Serial.print(cfg_.accel_cs_pin);
  Serial.print(" gyro_cs="); Serial.println(cfg_.gyro_cs_pin);
  int rc = imu_.begin();
  // The underlying library returns >0 on success (1), and negative codes on failures.
  if (rc <= 0) {
    Serial.print("BMI088: imu_.begin() returned rc="); Serial.println(rc);
    // Provide a hint about common failure ranges from the underlying library
    if (rc < 0 && rc > -1000) {
      Serial.println("BMI088: accel init failed (check accel CS pin/wiring)");
    } else if (rc <= -1000 && rc > -2000) {
      Serial.println("BMI088: gyro init failed (check gyro CS pin/wiring)");
    } else {
      Serial.println("BMI088: feature/config upload or post-init failed");
    }
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
  // Use the raw read helper to avoid duplicating sensor access logic
  if (!readRaw(out)) return false;
  out.ts_ms = now;
  last_read_ms_ = now;
  // apply calibration if available
  abbot::imu_cal::applyCalibrationToSample(out);
  return true;
}

bool BMI088Driver::readRaw(IMUSample &out) {
  // immediate raw read without timing guard; useful for calibration
  imu_.readSensor();
  out.ax = imu_.getAccelX_mss();
  out.ay = imu_.getAccelY_mss();
  out.az = imu_.getAccelZ_mss();
  out.gx = imu_.getGyroX_rads();
  out.gy = imu_.getGyroY_rads();
  out.gz = imu_.getGyroZ_rads();
  out.time_ps = imu_.getTime_ps();
  out.ts_ms = millis();
  return true;
}

} // namespace abbot
