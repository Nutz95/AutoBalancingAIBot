# Gamepad Input Specification

## ADDED Requirements

### Requirement: BLE HID Controller Discovery
The system SHALL scan for BLE HID devices advertising the HID Service UUID (0x1812) and connect to the first discovered gamepad controller.

#### Scenario: Xbox controller in pairing mode
- **WHEN** Xbox Wireless Controller is put in pairing mode (pairing button pressed)
- **THEN** ESP32 SHALL detect the device via BLE advertisement
- **AND** ESP32 SHALL store the device address for connection

#### Scenario: No HID device found
- **WHEN** BLE scan runs for 30 seconds without finding HID Service UUID
- **THEN** ESP32 SHALL log "No HID device found" to Serial
- **AND** ESP32 SHALL remain in scan loop

### Requirement: Secure BLE Connection
The system SHALL establish an encrypted BLE connection using just-works pairing (no PIN) and subscribe to HID Input Report notifications.

#### Scenario: Successful connection and encryption
- **WHEN** ESP32 connects to discovered HID device
- **THEN** ESP32 SHALL call `secureConnection()` to initiate encryption
- **AND** ESP32 SHALL wait for BLE_HS_EV_AUTH_COMPLETE event
- **AND** ESP32 SHALL discover HID Input Report characteristic (UUID 0x2A4D)
- **AND** ESP32 SHALL subscribe to notifications via CCCD write

#### Scenario: Encryption fails
- **WHEN** `secureConnection()` returns non-zero status
- **THEN** ESP32 SHALL log "Encryption failed" to Serial
- **AND** ESP32 SHALL disconnect and retry connection

### Requirement: HID Report Parsing
The system SHALL parse 16-byte HID Input Reports from Xbox Wireless Controller to extract left stick X and Y axis values.

#### Scenario: HID report received
- **WHEN** notification callback receives 16-byte data buffer
- **THEN** system SHALL extract X axis from bytes 0-1 as little-endian uint16
- **AND** system SHALL extract Y axis from bytes 2-3 as little-endian uint16
- **AND** system SHALL log raw values to Serial for debugging

#### Scenario: Invalid report size
- **WHEN** notification callback receives data buffer with length != 16
- **THEN** system SHALL log warning "Invalid HID report size"
- **AND** system SHALL ignore the report

### Requirement: Joystick Calibration and Normalization
The system SHALL normalize joystick axis values using calibrated center points (X=0x82F9, Y=0x72A7) and map to [-1.0, 1.0] range.

#### Scenario: Joystick at rest position
- **WHEN** raw X value is 0x82F9 (33529) and raw Y value is 0x72A7 (29351)
- **THEN** normalized x SHALL be 0.0
- **AND** normalized y SHALL be 0.0

#### Scenario: Joystick at maximum forward
- **WHEN** raw Y value is 0x0000 (0)
- **THEN** normalized y SHALL be approximately -0.896
- **AND** system SHALL apply deadzone before use

#### Scenario: Joystick at maximum right
- **WHEN** raw X value is 0xFFFF (65535)
- **THEN** normalized x SHALL be approximately 0.977
- **AND** system SHALL apply deadzone before use

### Requirement: Deadzone Filtering
The system SHALL apply a deadzone threshold of 0.12 normalized units to eliminate joystick drift and mechanical play.

#### Scenario: Joystick within deadzone
- **WHEN** normalized |x| < 0.12 AND normalized |y| < 0.12
- **THEN** both x and y SHALL be set to 0.0
- **AND** no motor commands SHALL be sent

#### Scenario: Joystick outside deadzone
- **WHEN** normalized x = 0.15 (outside deadzone)
- **THEN** x SHALL remain 0.15 (not zeroed)
- **AND** motor commands SHALL use the original value

### Requirement: Tank Drive Mapping
The system SHALL convert joystick input to tank drive motor commands using the formula: left = forward + turn, right = forward - turn, where forward = -y and turn = x * 0.6.

#### Scenario: Forward movement
- **WHEN** normalized y = -0.5 (stick pushed forward) and x = 0.0
- **THEN** left motor command SHALL be 0.5
- **AND** right motor command SHALL be 0.5

#### Scenario: Turn in place right
- **WHEN** normalized x = 0.8 (stick pushed right) and y = 0.0
- **THEN** left motor command SHALL be 0.48 (0.8 * 0.6)
- **AND** right motor command SHALL be -0.48
- **AND** robot SHALL spin clockwise

#### Scenario: Forward-right arc
- **WHEN** normalized y = -0.6 (forward) and x = 0.4 (right)
- **THEN** left motor command SHALL be 0.84 (0.6 + 0.4*0.6)
- **AND** right motor command SHALL be 0.36 (0.6 - 0.4*0.6)
- **AND** robot SHALL arc right while moving forward

#### Scenario: Command clamping
- **WHEN** tank drive calculation produces left = 1.3
- **THEN** left motor command SHALL be clamped to 1.0
- **AND** motor driver SHALL receive valid range [-1.0, 1.0]

### Requirement: Auto-Enable Motors on First Movement
The system SHALL automatically enable motors when joystick moves beyond threshold (|x| > 0.01 or |y| > 0.01) while motors are disabled.

#### Scenario: Motors initially disabled
- **WHEN** motors are disabled AND joystick moved with |x| > 0.01
- **THEN** system SHALL call `enableMotors()`
- **AND** system SHALL log "Motors enabled (auto)" to Serial
- **AND** subsequent motor commands SHALL be executed

#### Scenario: Motors already enabled
- **WHEN** motors are already enabled AND joystick moved
- **THEN** system SHALL NOT call `enableMotors()` again
- **AND** motor commands SHALL be sent normally

#### Scenario: Joystick below threshold
- **WHEN** joystick at rest with |x| = 0.005 and |y| = 0.008
- **THEN** system SHALL NOT enable motors
- **AND** no motor commands SHALL be sent

### Requirement: Auto-Disable Motors on Disconnect
The system SHALL automatically disable motors when BLE controller disconnects to prevent uncontrolled movement.

#### Scenario: Controller disconnects while motors enabled
- **WHEN** BLE connection is lost AND motors are enabled
- **THEN** system SHALL call `disableMotors()`
- **AND** system SHALL log "Controller disconnected, motors disabled" to Serial
- **AND** motors SHALL release torque

#### Scenario: Controller disconnects while motors disabled
- **WHEN** BLE connection is lost AND motors are already disabled
- **THEN** system SHALL NOT call `disableMotors()` again
- **AND** system SHALL log "Controller disconnected" to Serial

### Requirement: Direct Motor Control Integration
The system SHALL send motor velocity commands directly to motor driver API (bypassing serial command loop) for deterministic low-latency control.

#### Scenario: Motor command from joystick
- **WHEN** tank drive calculation produces left = 0.5, right = 0.3
- **THEN** system SHALL call `abbot::motor::setMotorCommand(8, 0.5)` for left motor
- **AND** system SHALL call `abbot::motor::setMotorCommand(7, 0.3)` for right motor
- **AND** motor driver SHALL execute commands within 5ms

#### Scenario: Zero command at rest
- **WHEN** joystick is within deadzone (x=0, y=0)
- **THEN** system SHALL call `setMotorCommand(8, 0.0)` for left motor
- **AND** system SHALL call `setMotorCommand(7, 0.0)` for right motor
- **AND** motors SHALL stop

### Requirement: Connection State Tracking
The system SHALL maintain connection state flag (`g_connected`) to track BLE controller connection status.

#### Scenario: Connection established
- **WHEN** BLE connection completes successfully
- **THEN** `g_connected` SHALL be set to true
- **AND** system SHALL log "BLE Connected" to Serial

#### Scenario: Connection lost
- **WHEN** BLE disconnection event occurs
- **THEN** `g_connected` SHALL be set to false
- **AND** system SHALL log "BLE Disconnected" to Serial
