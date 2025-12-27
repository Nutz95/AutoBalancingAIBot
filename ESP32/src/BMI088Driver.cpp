// BMI088Driver.cpp
#include "BMI088Driver.h"
#include "imu_calibration.h"
#include "logging.h"

namespace abbot {

BMI088Driver::BMI088Driver(const BMI088Config &cfg)
    : cfg_(cfg), imu_(SPI, cfg.accel_cs_pin, cfg.gyro_cs_pin) {}

bool BMI088Driver::begin() {
  // Initialize SPI and sensor
  SPI.begin();
  LOG_PRINTF(abbot::log::CHANNEL_DEFAULT, "BMI088: accel_cs=%d gyro_cs=%d\n",
             cfg_.accel_cs_pin, cfg_.gyro_cs_pin);
  int rc = imu_.begin();
  // The underlying library returns >0 on success (1), and negative codes on
  // failures.
  if (rc <= 0) {
    LOG_PRINTF(abbot::log::CHANNEL_DEFAULT,
               "BMI088: imu_.begin() returned rc=%d\n", rc);
    // Provide a hint about common failure ranges from the underlying library
    if (rc < 0 && rc > -1000) {
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT,
                  "BMI088: accel init failed (check accel CS pin/wiring)");
    } else if (rc <= -1000 && rc > -2000) {
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT,
                  "BMI088: gyro init failed (check gyro CS pin/wiring)");
    } else {
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT,
                  "BMI088: feature/config upload or post-init failed");
    }
    return false;
  }
  // Set ODR on sensor. The Bmi088 combined class only exposes ODR_2000HZ,
  // ODR_1000HZ, and ODR_400HZ. For lower sample rates we still configure
  // the sensor at 400Hz and throttle reads via sampling_interval_us().
  if (cfg_.sampling_hz >= 1000) {
    imu_.setOdr(Bmi088::ODR_1000HZ);
  } else {
    // 400Hz is the minimum combined ODR available
    imu_.setOdr(Bmi088::ODR_400HZ);
  }
  LOG_PRINTF(abbot::log::CHANNEL_DEFAULT, "BMI088: configured for %d Hz sampling\\n",
             cfg_.sampling_hz);
  last_read_us_ = micros();
  return true;
}

bool BMI088Driver::read(IMUSample &out) {
  unsigned long now_us = micros();
  unsigned long interval_us = cfg_.sampling_interval_us();
  if (interval_us == 0) {
    return false;
  }

  if ((now_us - last_read_us_) < interval_us) {
    return false; // not time yet
  }
  // Use the raw read helper to avoid duplicating sensor access logic
  if (!readRaw(out)) {
    return false;
  }
  out.ts_ms = millis();
  out.ts_us = now_us;
  last_read_us_ = now_us;
  // apply calibration if available
  abbot::imu_cal::applyCalibrationToSample(out);
  return true;
}

bool BMI088Driver::readRaw(IMUSample &out) {
  // immediate raw read without timing guard; useful for calibration
  imu_.readSensor();
  // Apply sensor axis signs from config to correct for physical mounting
  out.ax = cfg_.accel_sign_x * imu_.getAccelX_mss();
  out.ay = cfg_.accel_sign_y * imu_.getAccelY_mss();
  out.az = cfg_.accel_sign_z * imu_.getAccelZ_mss();
  out.gx = cfg_.gyro_sign_x * imu_.getGyroX_rads();
  out.gy = cfg_.gyro_sign_y * imu_.getGyroY_rads();
  out.gz = cfg_.gyro_sign_z * imu_.getGyroZ_rads();
  out.temp_C = imu_.getTemperature_C();
  out.time_ps = imu_.getTime_ps();
  out.ts_ms = millis();
  out.ts_us = micros();
  return true;
}

} // namespace abbot
