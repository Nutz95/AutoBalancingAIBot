// imu_calibration.h
#pragma once

#include "imu_drivers/IIMUDriver.h"
#include <stdint.h>

namespace abbot {
namespace imu_cal {

struct Calibration {
  float gyro_bias[3];
  float accel_offset[3];
  uint32_t version;
  bool valid;
};

// Load calibration from NVS; returns true if a valid calibration was loaded
bool loadCalibration(Calibration &out);
// Save calibration to NVS
bool saveCalibration(const Calibration &cal);

// Install a previously-loaded calibration into the module's runtime state
// so that `dumpCalibration()` and `applyCalibrationToSample()` use it.
void installCalibration(const Calibration &cal);

// Apply stored calibration to a sample (in-place)
void applyCalibrationToSample(struct abbot::IMUSample &s);

// Blocking calibration routines (they will sample the sensor N times)
// Return true on success and save the calibration to NVS.
bool startGyroCalibration(class abbot::IIMUDriver &driver,
                          int nSamples = 2000);
bool startAccelCalibration(class abbot::IIMUDriver &driver,
                           int nSamples = 2000);

// Dump / reset
void dumpCalibration();
void resetCalibration();

// State
bool isCalibrating();

} // namespace imu_cal
} // namespace abbot
