// serial_commands.h
#pragma once

#include <Arduino.h>
namespace abbot {
// Forward declaration
class BMI088Driver;

namespace serialcmds {

// Serial task entry (FreeRTOS task) -- pass a pointer to BMI088Driver as pv
void serialTaskEntry(void *pvParameters);

// Process one serial command (call from loop)
void processSerialOnce(abbot::BMI088Driver *driver);

// Process a single line of serial input (helpers for remote console).
// `driver` may be nullptr; commands that require the driver will be ignored
// if it is not available. This is safe to call from other modules (e.g. wifi).
void processSerialLine(abbot::BMI088Driver *driver, const String &line);

// Convenience: deliver a remote line to the serial command processor using
// the task's configured driver. Safe to call from the Wi‑Fi console.
void receiveRemoteLine(const String &line);

// Start the interactive numeric serial menu (prints the menu and makes it
// active). Pass a pointer to the `BMI088Driver` so calibration commands can be
// bound.
void startInteractiveMenu(abbot::BMI088Driver *driver);

} // namespace serialcmds
} // namespace abbot
