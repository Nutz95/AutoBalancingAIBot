// imu_calibration.h
#pragma once

#include <stdint.h>
#include "BMI088Driver.h"

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
bool startGyroCalibration(class abbot::BMI088Driver &driver, int nSamples = 2000);
bool startAccelCalibration(class abbot::BMI088Driver &driver, int nSamples = 2000);

// Dump / reset
void dumpCalibration();
void resetCalibration();

// State
bool isCalibrating();

// Serial task entry (FreeRTOS task) -- pass a pointer to BMI088Driver as pv
void serialTaskEntry(void *pvParameters);

} // namespace imu_cal
} // namespace abbot
