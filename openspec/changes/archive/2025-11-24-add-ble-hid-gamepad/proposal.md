# Change: Add BLE HID Gamepad Input

## Why

The robot currently lacks a practical manual control method. The existing HTML server interface requires a computer and network connection, making field testing and demonstrations cumbersome. A Bluetooth gamepad provides instant, wireless control with familiar ergonomics, enabling quick testing, demonstrations, and emergency manual takeover.

## What Changes

- Add BLE HID central client support using NimBLE-Arduino library
- Implement Xbox Wireless Controller pairing and HID report parsing
- Add calibrated joystick mapping to tank drive motor commands
- Implement auto-enable motors on first joystick movement
- Implement auto-disable motors on controller disconnect (safety)
- Direct motor control integration bypassing serial command loop
- Remove unused Bluepad32 (Bluetooth Classic) code incompatible with ESP32-S3

## Impact

**Affected specs:**
- `gamepad-input` (new capability - BLE HID gamepad integration)
- `motor-control` (existing - motor enable/disable state management)

**Affected code:**
- `ESP32/src/btle_hid.cpp` (new file - BLE HID client implementation)
- `ESP32/src/btle_hid.h` (new file - public API)
- `ESP32/src/main.cpp` (modified - simplified to BLE-only startup path)
- `ESP32/platformio.ini` (modified - added NimBLE-Arduino@^1.4.2 dependency)

**Dependencies:**
- Added: h2zero/NimBLE-Arduino@^1.4.2 (BLE stack)
- Removed: Bluepad32 conditional code (incompatible with ESP32-S3 BLE-only hardware)

**Hardware requirements:**
- ESP32-S3 with BLE support (BLE-only, no Bluetooth Classic)
- Xbox Wireless Controller (tested with Xbox One/Series controller)

**Breaking changes:**
- None (additive feature)
