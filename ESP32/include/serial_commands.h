// serial_commands.h
#pragma once

namespace abbot {
namespace serialcmds {

// Serial task entry (FreeRTOS task) -- pass a pointer to BMI088Driver as pv
void serialTaskEntry(void *pvParameters);

} // namespace serialcmds
} // namespace abbot
