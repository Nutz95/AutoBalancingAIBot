// BMI160Config.h
#pragma once

#include <stdint.h>

namespace abbot {

struct BMI160Config {
  // Protocol selection
  bool use_spi = true;

  // SPI pins for ESP32-S3 (SCK=4, MOSI=5, MISO=6, CS=7)
  // Note: GPIO 9-14 are reserved for internal PSRAM on this board.
  int spi_sck = 4;
  int spi_mosi = 5;
  int spi_miso = 6;
  int spi_cs = 7;

  // SPI Bus Speed (Hz). Default is 10MHz (10000000).
  // If stability issues or long cables, reduce to 5MHz (5000000) or 2MHz.
  uint32_t spi_speed_hz = 10000000;

  // I2C address fallback (if still needed)
  uint8_t i2c_addr = 0x69;
  int sda_pin = 11;
  int scl_pin = 12;

  // Sampling frequency in Hz (BMI160 can do 100, 200, 400, 800, 1600 Hz)
  // Higher frequency allows better PID performance.
  // Set to 500 to allow 1000/2 ticks (500Hz) and avoid the 333Hz trap of 400Hz.
  uint16_t sampling_hz = 500;

  // Sensor axis signs: +1 or -1 to correct for physical mounting orientation.
  // Note: BMI160 directions adjusted for balancing logic (Forward = Positive Pitch).
  int8_t accel_sign_x = -1;
  int8_t accel_sign_y = 1;
  int8_t accel_sign_z = -1;
  int8_t gyro_sign_x = 1;
  int8_t gyro_sign_y = -1;
  int8_t gyro_sign_z = 1;

  // Balancing logic orientation: 1 or -1
  // Defines if positive pitch rate (falling forward) matches positive gyro Y.
  // Set to 1 if Pitch increases when falling forward and Gyro Y is positive.
  int8_t pitch_rate_sign = 1;

  unsigned long sampling_interval_us() const {
    return (sampling_hz > 0) ? (1000000u / sampling_hz) : 0u;
  }
};

} // namespace abbot
