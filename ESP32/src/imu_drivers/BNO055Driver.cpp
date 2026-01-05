// BNO055Driver.cpp
#include "../../include/imu_drivers/BNO055Driver.h"
#include "../../include/imu_calibration.h"
#include "../../include/logging.h"

namespace abbot {

BNO055Driver::BNO055Driver(const BNO055Config &cfg)
    : cfg_(cfg), bno_(Adafruit_BNO055(-1, cfg.i2c_addr, &Wire)) {
}

bool BNO055Driver::begin() {
  // Enable internal pull-ups as a fallback (though external 4.7k-10k are better)
  pinMode(cfg_.sda_pin, INPUT_PULLUP);
  pinMode(cfg_.scl_pin, INPUT_PULLUP);
  delay(10);

  // Initialize I2C with configured pins and 100kHz for better stability
  if (!Wire.begin(cfg_.sda_pin, cfg_.scl_pin, 100000)) {
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "BNO055: Wire.begin() failed");
    return false;
  }

  // Increase timeout to handle BNO055 clock stretching
  Wire.setTimeOut(25); 

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
  last_read_us_ = now_us;

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

  // Read internal fusion data (Euler angles in degrees)
  imu::Vector<3> euler = bno_.getVector(Adafruit_BNO055::VECTOR_EULER);
  // BNO055 Euler: X=Heading, Y=Roll, Z=Pitch (depending on orientation)
  // For our balancer, we typically care about Pitch.
  // Note: BNO055 internal fusion uses its own coordinate system.
  out.fused_pitch = cfg_.gyro_sign_x * euler.z() * DEGREES_TO_RADIANS;
  out.fused_roll = cfg_.gyro_sign_y * euler.y() * DEGREES_TO_RADIANS;
  out.fused_yaw = euler.x() * DEGREES_TO_RADIANS;

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
