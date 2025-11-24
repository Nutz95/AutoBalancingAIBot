# Implementation Tasks

## 1. BLE Stack Integration
- [x] 1.1 Add NimBLE-Arduino@^1.4.2 dependency to platformio.ini
- [x] 1.2 Configure NimBLE security (BLE_HS_IO_NO_INPUT_OUTPUT, no PIN pairing)
- [x] 1.3 Initialize NimBLE device and configure callbacks

## 2. Controller Discovery and Connection
- [x] 2.1 Implement BLE scan for HID Service UUID (0x1812)
- [x] 2.2 Implement connection logic with explicit `secureConnection()` call
- [x] 2.3 Implement connection parameter negotiation and encryption handling
- [x] 2.4 Add connection state tracking (`g_connected` flag)

## 3. HID Report Handling
- [x] 3.1 Discover HID Input Report characteristic (0x2A4D)
- [x] 3.2 Subscribe to notifications with CCCD write
- [x] 3.3 Implement notification callback for 16-byte Xbox HID reports
- [x] 3.4 Parse little-endian uint16 X/Y axis values from bytes 0-3

## 4. Joystick Calibration
- [x] 4.1 Identify Xbox controller center values (X=0x82F9, Y=0x72A7)
- [x] 4.2 Implement normalization to [-1, 1] range using calibrated centers
- [x] 4.3 Apply deadzone threshold (0.12 normalized units)
- [x] 4.4 Document calibration values in code comments

## 5. Motor Control Integration
- [x] 5.1 Implement tank drive math (forward/turn to left/right motor commands)
- [x] 5.2 Apply turn sensitivity multiplier (0.6x)
- [x] 5.3 Clamp motor commands to [-1, 1] range
- [x] 5.4 Replace Serial-based commands with direct `setMotorCommand()` calls
- [x] 5.5 Add LEFT_MOTOR_ID and RIGHT_MOTOR_ID definitions from motor_config.h

## 6. Safety Features
- [x] 6.1 Implement auto-enable motors on first significant joystick movement
- [x] 6.2 Implement auto-disable motors on controller disconnect
- [x] 6.3 Add movement threshold check (|x|>0.01 or |y|>0.01)
- [x] 6.4 Log motor state changes to Serial

## 7. Code Cleanup
- [x] 7.1 Remove USE_BLUEPAD32 conditional compilation blocks
- [x] 7.2 Remove PS3 controller polling code (normAxis, pollPS3, g_ps3)
- [x] 7.3 Simplify main.cpp to single BLE HID path
- [x] 7.4 Update platformio.ini comments to reflect BLE-only design

## 8. Testing and Validation
- [x] 8.1 Verify BLE scan and connection with Xbox Wireless Controller
- [x] 8.2 Verify encryption and notification subscription success
- [x] 8.3 Confirm joystick data parsing and normalization accuracy
- [x] 8.4 Test auto-enable on first stick movement
- [x] 8.5 Test auto-disable on controller disconnect
- [x] 8.6 Physical motor testing: verify motors respond to joystick input
- [x] 8.7 Verify manual commands still work via Serial (`MOTOR SET`, `MOTOR STATUS`)

## 9. Documentation
- [x] 9.1 Document BLE HID architecture in design.md
- [x] 9.2 Document calibration process and values
- [x] 9.3 Document security/pairing flow
- [x] 9.4 Create spec delta for gamepad-input capability
- [x] 9.5 Validate OpenSpec change with strict mode
