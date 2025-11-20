// BMI088Config.h
#pragma once

#include <stdint.h>

namespace abbot {

struct BMI088Config {
  // Chip select pins for accel and gyro (SPI)
  uint8_t accel_cs_pin = 10; // GPIO10 (FSPICS0)
  uint8_t gyro_cs_pin = 14;  // GPIO14 (chip select2)

  // Sampling frequency in Hz (target read frequency)
  uint16_t sampling_hz = 200;

  // Convenience: compute interval in milliseconds
  unsigned long sampling_interval_ms() const {
    return (sampling_hz > 0) ? (1000u / sampling_hz) : 0u;
  }
};

} // namespace abbot
