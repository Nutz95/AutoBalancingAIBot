// SystemTasks.h
#pragma once

#include "BMI088Driver.h"
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

 #define ENABLE_DEBUG_LOGS

namespace abbot {

// Initialize and start system FreeRTOS tasks (IMU producer/consumer, serial, etc.)
// The producer polls the BMI088Driver and writes the latest sample to a single-slot queue.
// The consumer reads from the queue and prints or forwards samples.

bool startIMUTasks(BMI088Driver *driver);

// Allow calibration code to attach a temporary single-slot queue that the IMU producer
// will push samples into. Attach before starting calibration and detach/delete after.
void attachCalibrationQueue(QueueHandle_t q);
void detachCalibrationQueue();

// Accessors for the fused attitude (Madgwick). Safe to call from other tasks.
// Returned values are in radians (pitch) and rad/s (pitch rate).
// Get fused pitch and pitch rate.
// Units: `getFusedPitch()` returns pitch in radians. `getFusedPitchRate()` returns pitch rate in rad/s.
float getFusedPitch();
float getFusedPitchRate();

// TUNING CSV stream control. When enabled the device will print CSV lines to
// the serial console at each IMU sample containing: timestamp_ms,pitch_deg,pitch_rate_deg_s,left_motor_cmd,right_motor_cmd
// Use the serial command `TUNING START` / `TUNING STOP` to control from host.
void startTuningStream();
void stopTuningStream();
bool isTuningStreamActive();

} // namespace abbot
