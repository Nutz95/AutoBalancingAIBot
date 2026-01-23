#pragma once

#include <cstddef>
#include <cstdint>

// =============================================================================
// Controller Button Mapping Configuration
// =============================================================================
// Configure which HID report bytes and bits correspond to controller actions.
// HID reports are arrays of bytes; each button is identified by:
//   - Byte index (0-based position in the HID report)
//   - Bit mask (which bit(s) within that byte represent the button)
//
// To discover button mappings, enable BLE logging and press buttons on your
// controller. The HID report will be printed, showing which bytes change.
// Example log: "HID[16]: 28 80 25 7B 7B 7D 45 7B 00 00 00 00 00 00 08 00"
// When a button changes a byte from 00 to 08, that's byte index 14, mask 0x08.
// =============================================================================

namespace controller_config {
// --- Controller Identification ---
// The BLE name of the preferred controller to speed up connection.
constexpr const char* CONTROLLER_NAME = "Xbox Wireless Controller";
// --- Balance Toggle Button ---
// Button to start/stop balancing. Action triggers on button RELEASE (falling edge).
constexpr uint8_t BALANCE_TOGGLE_BYTE = 15;   // Byte index in HID report
constexpr uint8_t BALANCE_TOGGLE_MASK = 0x01; // Bit mask for the button

// --- Trim Calibrate Button ---
// Button to calibrate trim: captures current pitch as 0° reference and saves to NVS.
// Action triggers on button RELEASE (falling edge).
// Set to same byte/mask as another button if you want to disable this feature.
constexpr uint8_t TRIM_CALIBRATE_BYTE = 14;   // Byte index in HID report
constexpr uint8_t TRIM_CALIBRATE_MASK = 0x08; // Bit mask for the button (bit 3)

// --- Minimum HID Report Length ---
// Reject HID reports shorter than this to avoid reading invalid data.
constexpr size_t MIN_HID_REPORT_LENGTH = 16;

} // namespace controller_config
