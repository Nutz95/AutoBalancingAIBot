// BNO055Driver.cpp
#include "../../include/imu_drivers/BNO055Driver.h"
#include "../../include/imu_calibration.h"
#include "../../include/logging.h"

namespace abbot {

BNO055Driver::BNO055Driver(const BNO055Config &cfg)
    : cfg_(cfg), bno_(Adafruit_BNO055(-1, cfg.i2c_addr, &Wire)) {
}

bool BNO055Driver::begin() {
  LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "BNO055: starting initialization...");

  // 1. Hard I2C Bus Recovery
  // If a slave (BNO055) is stuck mid-transmission, it might be holding SDA low.
  // We toggle SCL 9 times to force the slave to release the SDA line.
  pinMode(cfg_.scl_pin, OUTPUT);
  pinMode(cfg_.sda_pin, INPUT_PULLUP);
  for (int i = 0; i < 9; i++) {
    digitalWrite(cfg_.scl_pin, LOW);
    delayMicroseconds(10);
    digitalWrite(cfg_.scl_pin, HIGH);
    delayMicroseconds(10);
  }

  // 2. Initialize Wire with timeout
  Wire.end(); 
  Wire.begin(cfg_.sda_pin, cfg_.scl_pin);
  Wire.setClock(400000); // 400kHz
  Wire.setTimeOut(25);   // 25ms timeout for all I2C operations

  // Give the sensor time to wake up
  delay(100);

  // Initialize BNO055 with retries to handle slow startup
  int retries = 5;
  bool success = false;
  while (retries > 0) {
    if (bno_.begin()) {
      success = true;
      break;
    }
    retries--;
    if (retries > 0) {
      LOG_PRINTF(abbot::log::CHANNEL_DEFAULT, "BNO055: bno.begin() failed, retrying... (%d left)\n", retries);
      delay(500);
    }
  }

  if (!success) {
    LOG_PRINTF(abbot::log::CHANNEL_DEFAULT, "BNO055: bno.begin() failed at addr 0x%02X after retries\n", cfg_.i2c_addr);
    return false;
  }

  // Use external crystal for better stability
  bno_.setExtCrystalUse(true);

  LOG_PRINTF(abbot::log::CHANNEL_DEFAULT, "BNO055: initialized at 0x%02X (SDA=%d, SCL=%d)\n",
             cfg_.i2c_addr, cfg_.sda_pin, cfg_.scl_pin);
  
  last_read_us_ = micros();
  return true;
}

bool BNO055Driver::read(IMUSample &out) {
  unsigned long now_us = micros();
  unsigned long interval_us = cfg_.sampling_interval_us();
  
  if (interval_us > 0 && (now_us - last_read_us_) < interval_us) {
    return false;
  }

  if (!readRaw(out)) {
    return false;
  }

  out.ts_ms = millis();
  out.ts_us = now_us;
  
  // Use fixed-interval step to maintain average frequency and avoid drift/jitter accumulation.
  // If we are significantly late (> 2 intervals), reset the baseline to 'now'.
  if (now_us - last_read_us_ > 2 * interval_us) {
    last_read_us_ = now_us;
  } else {
    last_read_us_ += interval_us;
  }

  // Apply software calibration if available (though BNO055 has internal cal)
  abbot::imu_cal::applyCalibrationToSample(out);
  
  return true;
}

bool BNO055Driver::readRaw(IMUSample &out) {
  // Read accelerometer (m/s^2)
  imu::Vector<3> accel = bno_.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  out.ax = cfg_.accel_sign_x * accel.x();
  out.ay = cfg_.accel_sign_y * accel.y();
  out.az = cfg_.accel_sign_z * accel.z();

  // Read gyroscope (rad/s)
  imu::Vector<3> gyro = bno_.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  const float DEGREES_TO_RADIANS = 3.1415926535f / 180.0f;
  out.gx = cfg_.gyro_sign_x * gyro.x() * DEGREES_TO_RADIANS;
  out.gy = cfg_.gyro_sign_y * gyro.y() * DEGREES_TO_RADIANS;
  out.gz = cfg_.gyro_sign_z * gyro.z() * DEGREES_TO_RADIANS;

  // Use Quaternion for more robust orientation extraction
  // Euler angles from BNO055 are prone to gimbal lock and depend on mounting.
  imu::Quaternion quat = bno_.getQuat();
  
  // Extract Pitch from quaternion (Rotation around Robot-Y axis)
  // formula: pitch = asin(2 * (qw*qy - qx*qz))
  double qw = quat.w();
  double qx = quat.x();
  double qy = quat.y();
  double qz = quat.z();
  
  float sinp = 2.0f * (float)(qw * qy - qx * qz);
  float pitch_rad;
  if (fabsf(sinp) >= 1) {
    pitch_rad = copysignf(M_PI / 2.0f, sinp); // use 90 degrees if out of range
  } else {
    pitch_rad = asinf(sinp);
  }

  // Offset if mounted upside down (Z pointing down instead of up)
  // If az is positive when vertical, the sensor is likely upside down.
  const float PI_VAL = 3.1415926535f;
  float final_pitch_rad = pitch_rad;

  // Normalisation
  while (final_pitch_rad > PI_VAL) final_pitch_rad -= 2.0f * PI_VAL;
  while (final_pitch_rad < -PI_VAL) final_pitch_rad += 2.0f * PI_VAL;

  out.fused_pitch = final_pitch_rad;
  
  // Basic Euler extraction for Roll/Yaw (not used for control)
  out.fused_roll = atan2(2.0f * (qw * qx + qy * qz), 1.0f - 2.0f * (qx * qx + qy * qy));
  out.fused_yaw = atan2(2.0f * (qw * qz + qx * qy), 1.0f - 2.0f * (qy * qy + qz * qz));

  // Basic sanity check: if all values are exactly zero, it's likely an I2C read error
  if (out.ax == 0.0f && out.ay == 0.0f && out.az == 0.0f && 
      out.gx == 0.0f && out.gy == 0.0f && out.gz == 0.0f) {
    return false;
  }

  // Read temperature
  out.temperatureCelsius = (float)bno_.getTemp();
  
  // BNO055 doesn't provide a high-res sensor timestamp in the same way as BMI088
  out.time_ps = 0;
  out.ts_ms = millis();
  out.ts_us = micros();

  return true;
}

bool BNO055Driver::isCalibrated() const {
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  // const_cast because getCalibration is not marked const in the library
  const_cast<Adafruit_BNO055&>(bno_).getCalibration(&system, &gyro, &accel, &mag);
  
  // Log calibration status periodically if not fully calibrated
  static uint32_t last_cal_log = 0;
  if (millis() - last_cal_log > 5000) {
    LOG_PRINTF(abbot::log::CHANNEL_IMU, "BNO055 Calib: Sys=%d G=%d A=%d M=%d\n", system, gyro, accel, mag);
    last_cal_log = millis();
  }

  // Consider "calibrated" if gyro and accel are at least level 2
  return (gyro >= 2 && accel >= 2);
}

uint16_t BNO055Driver::getSamplingHz() const {
  return cfg_.sampling_hz;
}

const char* BNO055Driver::getDriverName() const {
  return "BNO055";
}

} // namespace abbot
