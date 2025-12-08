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

  // Sampling frequency in Hz (target read frequency)
  uint16_t sampling_hz = 200;

  // Convenience: compute interval in milliseconds
  unsigned long sampling_interval_ms() const {
    return (sampling_hz > 0) ? (1000u / sampling_hz) : 0u;
  }
};

} // namespace abbot
