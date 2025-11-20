// IMUTasks.h
#pragma once

#include "BMI088Driver.h"

namespace abbot {

// Initialize and start IMU producer and consumer FreeRTOS tasks.
// The producer polls the BMI088Driver and writes the latest sample to a single-slot queue.
// The consumer reads from the queue and calls a user-provided callback (for now, prints to Serial).

bool startIMUTasks(BMI088Driver *driver);

} // namespace abbot
