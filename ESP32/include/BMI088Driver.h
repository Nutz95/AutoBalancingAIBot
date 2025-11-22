// BMI088Driver.h
#pragma once

#include "../config/BMI088Config.h"
#include <stdint.h>
#include <Arduino.h>
#include <SPI.h>
#include <BMI088.h>

namespace abbot {

struct IMUSample {
  float ax, ay, az; // m/s^2
  float gx, gy, gz; // rad/s
  uint64_t time_ps; // sensor timestamp in picoseconds (if available)
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

private:
  BMI088Config cfg_;
  Bmi088 imu_ = Bmi088(SPI, 0, 0); // placeholder will be reinitialized in begin()
  unsigned long last_read_ms_ = 0;
};

} // namespace abbot
