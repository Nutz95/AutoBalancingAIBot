// BMI088Config.h (moved to config/)
#pragma once

#include <stdint.h>

namespace abbot {

struct BMI088Config {
  // Chip select pins for accel and gyro (SPI)
  // Note: accel and gyro CS pins were swapped previously; defaults corrected below
  // Default CS pin mapping (change to match your wiring):
  // - accel (BMI088 accel chip id = 0x1E) -> GPIO4
  // - gyro  (BMI088 gyro  chip id = 0x0F) -> GPIO5
  uint8_t accel_cs_pin = 4; // GPIO4 (chip select for accel)
  uint8_t gyro_cs_pin  = 5; // GPIO5 (chip select for gyro)

  // SPI Bus Speed (Hz). Default is 10MHz (10000000).
  uint32_t spi_speed_hz = 10000000;

  // Sampling frequency in Hz (target read frequency)
  // Increased to 400Hz for faster control loop and better gyro integration
  uint16_t sampling_hz = 400;

  // Sensor axis signs: +1 or -1 to correct for physical mounting orientation.
  // These are applied at the driver level BEFORE calibration and mapping.
  // If sensor reads -9.8 m/s² on az when Z points up, set accel_sign_z = -1.
  int8_t accel_sign_x = 1;
  int8_t accel_sign_y = -1;
  int8_t accel_sign_z = -1;  // BMI088 mounted upside-down: Z reads negative when up
  int8_t gyro_sign_x = 1;
  int8_t gyro_sign_y = -1;
  int8_t gyro_sign_z = -1;   // Match accel Z inversion for consistency

  // Balancing logic orientation: 1 or -1
  int8_t pitch_rate_sign = 1;

  // Convenience: compute interval in milliseconds (integer, may lose precision)
  unsigned long sampling_interval_ms() const {
    return (sampling_hz > 0) ? (1000u / sampling_hz) : 0u;
  }

  // Convenience: compute interval in microseconds (precise for non-integer ms)
  // 400Hz -> 2500µs, 200Hz -> 5000µs, 500Hz -> 2000µs
  unsigned long sampling_interval_us() const {
    return (sampling_hz > 0) ? (1000000u / sampling_hz) : 0u;
  }
  // Note: the driver may program the sensor ODR to the nearest supported
  // combined ODR (library exposes minimum combined ODR = 400Hz). When the
  // requested `sampling_hz` is lower than the sensor's programmed ODR the
  // driver will throttle reads using `sampling_interval_us()` to provide the
  // configured effective sample rate.
};

} // namespace abbot
