// BMI088Driver.h
#pragma once

#include "../config/BMI088Config.h"
#include <Arduino.h>
#include <BMI088.h>
#include <SPI.h>
#include <stdint.h>

namespace abbot {

struct IMUSample {
  float ax, ay, az;    // m/s^2
  float gx, gy, gz;    // rad/s
  float temp_C;        // sensor temperature Celsius (from accel temp register)
  uint64_t time_ps;    // sensor timestamp in picoseconds (if available)
  unsigned long ts_ms; // host timestamp (millis)
};

class BMI088Driver {
public:
  explicit BMI088Driver(const BMI088Config &cfg);
  bool begin();
  // read latest sample from sensor (returns true if sample read)
  bool read(IMUSample &out);
  // raw immediate read (no timing guard, no calibration applied)
  bool readRaw(IMUSample &out);

  // Accessor for configured sampling frequency (Hz)
  uint16_t getSamplingHz() const { return cfg_.sampling_hz; }

private:
  BMI088Config cfg_;
  Bmi088 imu_ =
      Bmi088(SPI, 0, 0); // placeholder will be reinitialized in begin()
  unsigned long last_read_ms_ = 0;
};

} // namespace abbot
