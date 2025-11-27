// serial_commands.h
#pragma once

namespace abbot {
// Forward declaration
class BMI088Driver;

namespace serialcmds {

// Serial task entry (FreeRTOS task) -- pass a pointer to BMI088Driver as pv
void serialTaskEntry(void *pvParameters);

// Process one serial command (call from loop)
void processSerialOnce(abbot::BMI088Driver *driver);

// Start the interactive numeric serial menu (prints the menu and makes it active).
// Pass a pointer to the `BMI088Driver` so calibration commands can be bound.
void startInteractiveMenu(abbot::BMI088Driver *driver);

} // namespace serialcmds
} // namespace abbot
