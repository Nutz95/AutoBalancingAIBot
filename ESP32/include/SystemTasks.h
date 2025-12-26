// SystemTasks.h
#pragma once

#include "BMI088Driver.h"
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#define ENABLE_DEBUG_LOGS
// #define ENABLE_IMU_DEBUG_LOGS

namespace abbot {

// Initialize and start system FreeRTOS tasks (IMU producer/consumer, serial,
// etc.) The producer polls the BMI088Driver and writes the latest sample to a
// single-slot queue. The consumer reads from the queue and prints or forwards
// samples.

bool startIMUTasks(BMI088Driver *driver);

// Allow calibration code to attach a temporary single-slot queue that the IMU
// producer will push samples into. Attach before starting calibration and
// detach/delete after.
void attachCalibrationQueue(QueueHandle_t q);
void detachCalibrationQueue();

// Accessors for the fused attitude (Madgwick). Safe to call from other tasks.
// Returned values are in radians (pitch) and rad/s (pitch rate).
// Get fused pitch and pitch rate.
// Units: `getFusedPitch()` returns pitch in radians. `getFusedPitchRate()`
// returns pitch rate in rad/s.
float getFusedPitch();
float getFusedPitchRate();

// TUNING CSV stream control. When enabled the device will print CSV lines to
// the serial console at each IMU sample containing:
// timestamp_ms,pitch_deg,pitch_rate_deg_s,left_motor_cmd,right_motor_cmd Use
// the serial command `TUNING START` / `TUNING STOP` to control from host.

// Request a warm-up period (seconds) for Madgwick filter before emitting tuning
// CSV. During warm-up the filter will keep processing samples but tuning output
// will be suppressed. Callers (e.g. tuning_capture) may call this when a
// capture starts.
void requestTuningWarmupSeconds(float seconds);

// Print Madgwick filter and consumer diagnostics to the default log channel.
// Useful to inspect filter state just before starting the balancer.
void printMadgwickDiagnostics();

// Reinitialize the filter's orientation from the current accelerometer reading.
// Call this at balance START to instantly match the filter to the actual
// robot orientation, avoiding slow convergence when beta is low.
void reinitFilterFromAccel();

// Fusion readiness helpers
// Returns true when Madgwick has been seeded and gyro bias initialized
bool isFusionReady();

// Remaining warmup samples (0 when no warmup in progress)
unsigned long getFusionWarmupRemaining();

// Runtime filter selection helpers are provided by `filter_manager.h`.

// Balancer control (PID-based). Control APIs are provided by
// `abbot::balancer::controller`; callers should invoke controller methods
// directly (e.g. `abbot::balancer::controller::start(...)`).

} // namespace abbot
