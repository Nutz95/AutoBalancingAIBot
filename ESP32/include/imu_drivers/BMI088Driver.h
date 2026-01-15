// BMI088Driver.h
#pragma once

#include "../../config/imu_configs/BMI088Config.h"
#include "IIMUDriver.h"
#include <Arduino.h>
#include <BMI088.h>
#include <SPI.h>
#include <stdint.h>

namespace abbot {

// BMI088Driver: thin wrapper around the Bmi088 sensor library.
// Note: the combined `Bmi088::setOdr` API exposes a limited set of
// combined ODR values (2000, 1000 and 400 Hz). To support arbitrary
// configured sampling rates the driver programs the sensor to the
// nearest supported ODR (minimum combined ODR is 400Hz) and throttles
// reads using microsecond timing (`micros()`) to achieve the requested
// sampling frequency via `sampling_interval_us()` in the config.
class BMI088Driver : public IIMUDriver {
public:
  explicit BMI088Driver(const BMI088Config &cfg);
  virtual ~BMI088Driver() {
  }

  bool begin() override;
  // read latest sample from sensor (returns true if sample read)
  bool read(IMUSample &out) override;
  // raw immediate read (no timing guard, no calibration applied)
  bool readRaw(IMUSample &out) override;

  // Accessor for configured sampling frequency (Hz)
  uint16_t getSamplingHz() const override;

  const char* getDriverName() const override;

  int8_t getPitchRateSign() const override {
    return cfg_.pitch_rate_sign;
  }

private:
  BMI088Config cfg_;
  Bmi088 imu_ =
      Bmi088(SPI, 0, 0); // placeholder will be reinitialized in begin()
  unsigned long last_read_us_ = 0;  // microsecond timing for precise sampling
};

} // namespace abbot
