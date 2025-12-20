// btle_hid.h -- simple BLE HID central prototype to read gamepad reports
#pragma once

namespace abbot {
namespace btle_hid {
// Start the BLE HID client (non-blocking). Will log report bytes and
// translate simple joystick reports to `MOTOR SET` serial commands.
void begin();
} // namespace btle_hid
} // namespace abbot
