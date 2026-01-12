// BNO055Config.h
#pragma once

#include <stdint.h>

namespace abbot {

struct BNO055Config {
  // I2C address (default 0x28, alternative 0x29 if ADR pin is high)
  uint8_t i2c_addr = 0x29;

  // I2C pins for ESP32-S3 (default SDA=8, SCL=9)
  int sda_pin = 8;
  int scl_pin = 9;

  // Sampling frequency in Hz (BNO055 internal fusion is 100Hz)
  uint16_t sampling_hz = 100;

  // Sensor axis signs: +1 or -1 to correct for physical mounting orientation.
  // BNO055 has internal axis remap, but we can also apply signs here.
  int8_t accel_sign_x = 1;
  int8_t accel_sign_y = 1;
  int8_t accel_sign_z = -1; // Invert Z for standard gravity orientation
  int8_t gyro_sign_x = 1;   // Correct for mounting orientation
  int8_t gyro_sign_y = 1;
  int8_t gyro_sign_z = 1;

  unsigned long sampling_interval_us() const {
    return (sampling_hz > 0) ? (1000000u / sampling_hz) : 0u;
  }
};

} // namespace abbot
