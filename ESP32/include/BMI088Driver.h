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
  unsigned long ts_us; // host timestamp (micros)
};

// BMI088Driver: thin wrapper around the Bmi088 sensor library.
// Note: the combined `Bmi088::setOdr` API exposes a limited set of
// combined ODR values (2000, 1000 and 400 Hz). To support arbitrary
// configured sampling rates the driver programs the sensor to the
// nearest supported ODR (minimum combined ODR is 400Hz) and throttles
// reads using microsecond timing (`micros()`) to achieve the requested
// sampling frequency via `sampling_interval_us()` in the config.
class BMI088Driver {
public:
  explicit BMI088Driver(const BMI088Config &cfg);
  bool begin();
  // read latest sample from sensor (returns true if sample read)
  bool read(IMUSample &out);
  // raw immediate read (no timing guard, no calibration applied)
  bool readRaw(IMUSample &out);

  // Accessor for configured sampling frequency (Hz)
  uint16_t getSamplingHz() const {
    return cfg_.sampling_hz;
  }

private:
  BMI088Config cfg_;
  Bmi088 imu_ =
      Bmi088(SPI, 0, 0); // placeholder will be reinitialized in begin()
  unsigned long last_read_us_ = 0;  // microsecond timing for precise sampling
};

} // namespace abbot
