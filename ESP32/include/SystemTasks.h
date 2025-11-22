// SystemTasks.h
#pragma once

#include "BMI088Driver.h"
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

// #define ENABLE_DEBUG_LOGS

namespace abbot {

// Initialize and start system FreeRTOS tasks (IMU producer/consumer, serial, etc.)
// The producer polls the BMI088Driver and writes the latest sample to a single-slot queue.
// The consumer reads from the queue and prints or forwards samples.

bool startIMUTasks(BMI088Driver *driver);

// Allow calibration code to attach a temporary single-slot queue that the IMU producer
// will push samples into. Attach before starting calibration and detach/delete after.
void attachCalibrationQueue(QueueHandle_t q);
void detachCalibrationQueue();

} // namespace abbot
