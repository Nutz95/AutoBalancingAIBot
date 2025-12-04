// SystemTasks.h
#pragma once

#include "BMI088Driver.h"
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

 #define ENABLE_DEBUG_LOGS
 //#define ENABLE_IMU_DEBUG_LOGS

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

// Request a warm-up period (seconds) for Madgwick filter before emitting tuning CSV.
// During warm-up the filter will keep processing samples but tuning output will be suppressed.
// Callers (e.g. tuning_capture) may call this when a capture starts.
void requestTuningWarmupSeconds(float seconds);

// Balancer control (PID-based). Start/stop the balancer which will use the
// runtime Madgwick outputs to compute motor commands via the PID controller.
// These are safe to call from the serial command task.
void startBalancer();
void stopBalancer();
bool isBalancerActive();
// Set PID gains (kp, ki, kd). Values are in controller units where pitch is radians.
void setBalancerGains(float kp, float ki, float kd);
// Retrieve current balancer gains (returned via reference). Units: pitch in radians.
void getBalancerGains(float &kp, float &ki, float &kd);

// Deadband (minimum commanded absolute motor value to overcome static friction).
float getBalancerDeadband();
void setBalancerDeadband(float db);

// Start an interactive/calibration routine for deadband (may be a stub).
void calibrateBalancerDeadband();

} // namespace abbot
