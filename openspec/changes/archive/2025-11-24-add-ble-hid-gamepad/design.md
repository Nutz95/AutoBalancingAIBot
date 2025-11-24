# Design: BLE HID Gamepad Input

## Context

The robot needs practical manual control for field testing, demonstrations, and emergency takeover. The existing HTML server requires a computer and network connection, making it unsuitable for quick testing scenarios. A Bluetooth gamepad provides familiar ergonomics with instant wireless connectivity.

**Constraints:**
- ESP32-S3 hardware is BLE-only (no Bluetooth Classic support)
- Existing motor control API uses velocity commands (-1.0 to 1.0 range)
- Serial command interface must remain available for manual debugging
- Controller disconnection must safely disable motors

**Stakeholders:**
- Developer testing motor control algorithms
- Field testing and demonstrations
- Future autonomous mode switching (gamepad to autonomous)

## Goals / Non-Goals

**Goals:**
- Support Xbox Wireless Controller via BLE HID
- Auto-enable motors on first joystick movement
- Auto-disable motors on controller disconnect (safety)
- Tank drive control with left stick (forward/back + turn)
- Zero configuration pairing (no PIN required)

**Non-Goals:**
- Button mapping for mode switching (future work)
- Support for multiple controller types (PS4, 8BitDo, etc.)
- Rate limiting or packet timeout (future optimization)
- Gyroscope or advanced input features

## Decisions

### Decision 1: NimBLE over Bluedroid
**What:** Use NimBLE-Arduino library instead of ESP32 Bluedroid stack.

**Why:**
- NimBLE has cleaner central client API for BLE scanning and connections
- h2zero/NimBLE-Arduino is actively maintained with ESP32-S3 support
- Previous Bluedroid attempts had silent notification callback failures
- NimBLE explicit security flow (`secureConnection()`) worked reliably

**Alternatives considered:**
- ESP32 Bluedroid: Native stack but complex API, notification callbacks didn't fire
- Bluepad32: Bluetooth Classic only, incompatible with ESP32-S3 BLE-only hardware

### Decision 2: BLE HID Profile over Custom Service
**What:** Connect to standard HID Service (UUID 0x1812) instead of custom GATT service.

**Why:**
- Xbox controller exposes standard HID Input Report characteristic (0x2A4D)
- No custom firmware required on controller side
- Standard 16-byte HID report format well-documented
- Works with any HID-compliant gamepad (future expansion)

**Alternatives considered:**
- Custom GATT service: Would require custom controller firmware (not feasible)

### Decision 3: No-PIN Pairing (BLE_HS_IO_NO_INPUT_OUTPUT)
**What:** Configure NimBLE security with no PIN input/output capability.

**Why:**
- ESP32 has no physical PIN input mechanism (no keyboard, no display)
- Xbox controller supports just-works pairing (no PIN required)
- Security risk acceptable for local robot control (not transmitting sensitive data)

**Alternatives considered:**
- Fixed PIN: Controller doesn't support fixed PIN mode
- Display-only: ESP32 has no display for showing PIN

### Decision 4: Explicit `secureConnection()` Call After Connect
**What:** Call `pClient->secureConnection()` immediately after successful connection.

**Why:**
- Xbox controller sends dummy 0x00 HID reports before encryption established
- Notifications don't contain real button data until secure link negotiated
- NimBLE requires explicit encryption initiation for central role

**Trade-off:**
- Adds connection latency (~500ms encryption handshake)
- But eliminates silent notification failures observed in earlier attempts

### Decision 5: Calibrated Joystick Center Values
**What:** Use measured center values (X=0x82F9, Y=0x72A7) instead of standard 0x8000.

**Why:**
- Xbox controller's actual rest position observed at X=33529, Y=29351
- Using standard 0x8000 (32768) caused constant 0.02 drift in normalized output
- Controller doesn't self-calibrate on startup

**Calibration method:**
- Logged raw uint16 values with controller at rest
- Averaged over 10 seconds to account for noise
- Hardcoded in `notifyCallback()` for consistent zero point

**Trade-off:**
- Calibration specific to this Xbox controller unit
- Different controllers may need recalibration (future: auto-calibration on first connect)

### Decision 6: Direct Motor Control API Calls
**What:** Call `abbot::motor::setMotorCommand(id, value)` directly instead of sending Serial commands.

**Why:**
- Serial command loop (`processSerialOnce()`) must be called explicitly in `loop()`
- BLE notifications arrive asynchronously in callback context
- Sending serial strings and parsing adds 5-10ms latency
- Direct API calls provide deterministic <1ms response time

**Alternatives considered:**
- Serial commands: Tried initially but motors didn't respond (serial loop not called in BLE path)
- Message queue: Unnecessary complexity for simple velocity commands

### Decision 7: Auto-Enable Motors on First Movement
**What:** Automatically enable motors when joystick moves beyond threshold (0.01 normalized).

**Why:**
- User shouldn't need to send manual `MOTOR ENABLE` command before driving
- Joystick movement is clear intent signal (no accidental enable)
- Reduces friction for quick testing and demonstrations

**Threshold rationale:**
- 0.01 is well above deadzone (0.12) but below accidental touch
- Filters controller drift and noise
- Allows safe connection without immediate motor activation

### Decision 8: Tank Drive Mapping
**What:** Left stick Y = forward/back, left stick X = turn with 0.6 sensitivity multiplier.

**Math:**
```
forward = -y  (inverted: forward is negative Y on Xbox stick)
turn = x * 0.6
left_motor = forward + turn
right_motor = forward - turn
clamp(left_motor, -1.0, 1.0)
clamp(right_motor, -1.0, 1.0)
```

**Why:**
- Tank drive is intuitive for differential drive robots
- 0.6 turn multiplier provides responsive turning without oversensitivity
- Clamping ensures motor commands stay in valid range

**Alternatives considered:**
- Arcade drive (separate forward/turn sticks): Less intuitive for single-stick control
- 1.0 turn multiplier: Too sensitive, caused spinning in place at high forward speeds

### Decision 9: Deadzone of 0.12 Normalized Units
**What:** Ignore joystick input when |x| < 0.12 and |y| < 0.12.

**Why:**
- Xbox controller has mechanical play causing ~0.05-0.10 drift at rest
- After calibration, residual noise observed at 0.02-0.08 range
- 0.12 threshold eliminates drift while preserving low-speed control

**Trade-off:**
- Reduces effective range from [-1, 1] to [-0.88, 0.88]
- Acceptable loss given smooth zero point is critical for safety

## Architecture

### Component Diagram
```
┌─────────────────────────────────────────────────────────────┐
│ Xbox Wireless Controller                                    │
│ - HID Service (0x1812)                                      │
│ - Input Report Characteristic (0x2A4D)                      │
│ - 16-byte reports @ ~50Hz                                   │
└────────────────┬────────────────────────────────────────────┘
                 │ BLE Notifications
                 ▼
┌─────────────────────────────────────────────────────────────┐
│ NimBLE-Arduino Stack (ESP32-S3)                             │
│ - Central role, scan/connect/pair                           │
│ - Security: BLE_HS_IO_NO_INPUT_OUTPUT (no PIN)              │
│ - Connection params: 12-12ms interval, 0 latency, 100 TO    │
└────────────────┬────────────────────────────────────────────┘
                 │ notifyCallback()
                 ▼
┌─────────────────────────────────────────────────────────────┐
│ btle_hid.cpp::notifyCallback()                              │
│ 1. Parse 16-byte HID report (bytes 0-3: X/Y uint16)        │
│ 2. Normalize with calibrated centers (0x82F9, 0x72A7)      │
│ 3. Apply deadzone (0.12)                                    │
│ 4. Tank drive math (forward + turn → left/right)           │
│ 5. Auto-enable motors on first movement                     │
│ 6. Call motor driver API directly                           │
└────────────────┬────────────────────────────────────────────┘
                 │ setMotorCommand(id, value)
                 ▼
┌─────────────────────────────────────────────────────────────┐
│ Motor Driver (motor_driver.cpp)                             │
│ - Left motor (ID 8), Right motor (ID 7)                     │
│ - Velocity mode (WriteSpe API)                              │
│ - Enable/disable state management                           │
└─────────────────────────────────────────────────────────────┘
```

### Data Flow: HID Report → Motor Commands
```
1. Controller sends 16-byte HID report
   Example: 0xF9 0x82 0xA7 0x72 ... (X=0x82F9, Y=0x72A7)

2. Parse to raw uint16
   rawX = (uint16_t)(data[1] << 8 | data[0]) = 33529
   rawY = (uint16_t)(data[3] << 8 | data[2]) = 29351

3. Normalize with calibration
   x = (rawX - 33529) / 32768.0 = 0.0
   y = (rawY - 29351) / 32768.0 = 0.0

4. Apply deadzone
   if (|x| < 0.12 && |y| < 0.12) → x=0, y=0

5. Tank drive math
   forward = -y = 0.0
   turn = x * 0.6 = 0.0
   left = forward + turn = 0.0
   right = forward - turn = 0.0

6. Call motor API
   setMotorCommand(8, 0.0)  // Left motor
   setMotorCommand(7, 0.0)  // Right motor
```

### State Machine: Connection and Motor Control
```
┌─────────────┐
│ Disconnected│
│ Motors OFF  │
└──────┬──────┘
       │ begin() → scan
       ▼
┌─────────────┐
│  Scanning   │
│ Motors OFF  │
└──────┬──────┘
       │ Found HID device
       ▼
┌─────────────┐
│ Connecting  │
│ Motors OFF  │
└──────┬──────┘
       │ secureConnection()
       ▼
┌─────────────┐
│  Connected  │
│  Encrypted  │
│ Motors OFF  │
└──────┬──────┘
       │ First stick movement (>0.01)
       ▼
┌─────────────┐
│   Active    │
│ Motors ON   │
│ Receiving   │
└──────┬──────┘
       │ Controller disconnect
       ▼
┌─────────────┐
│ Disconnected│
│ Motors OFF  │
└─────────────┘
```

## Security

### Pairing Flow
1. ESP32 starts BLE scan filtering for HID Service UUID (0x1812)
2. User puts Xbox controller in pairing mode (press pairing button)
3. ESP32 detects controller advertisement and initiates connection
4. NimBLE performs just-works pairing (no PIN exchange)
5. ESP32 calls `secureConnection()` to establish encryption
6. Controller sends BLE_HS_EV_AUTH_COMPLETE event (status 0 = success)
7. Encrypted link established, real HID data flows

### Security Posture
- **Encryption:** BLE link-layer encryption active after pairing (AES-CCM)
- **Pairing persistence:** ESP32 stores bonding keys for reconnection without re-pairing
- **Attack surface:** Local wireless range only (~10 meters), no Internet exposure
- **Mitigation:** No sensitive data transmitted (motor velocity commands only)

**Acceptable risks:**
- No PIN prevents MITM attacks during initial pairing
- Acceptable for robot control application (not financial/health data)
- Physical proximity required (BLE range limited)

## Performance

### Latency Budget
- **Controller sampling:** ~50Hz (20ms period) - Xbox controller native rate
- **BLE transmission:** 12ms connection interval (configured)
- **Notification callback:** <1ms (direct motor API call)
- **Motor driver:** <5ms (STS servo UART command)
- **Total latency:** ~38ms (controller → motor response)

**Acceptable for manual control:**
- Human reaction time: ~200ms
- 38ms latency imperceptible to operator
- No rate limiting required for current implementation

### Memory Footprint
- **Flash:** 595KB total (BLE stack + application code)
- **RAM:** ~30KB (NimBLE stack buffers + application globals)
- **Stack usage:** ~4KB (notification callback with locals)

**No memory constraints on ESP32-S3:**
- 8MB flash, 512KB SRAM available
- BLE stack overhead acceptable

## Migration Plan

### Deployment Steps
1. Upload firmware to ESP32-S3 via PlatformIO
2. Power cycle robot (ESP32 starts BLE scan automatically)
3. Put Xbox controller in pairing mode (press pairing button)
4. ESP32 connects automatically (logs "BLE Connected" to Serial)
5. Move joystick to enable motors (logs "Motors enabled" to Serial)
6. Control robot with left stick (Y=forward/back, X=turn)

### Rollback Plan
If BLE HID integration fails:
1. Revert to previous commit (before NimBLE addition)
2. Re-upload firmware
3. Fall back to HTML server control via network

### Testing Checklist
- [x] BLE scan detects Xbox controller in pairing mode
- [x] Connection establishes and encryption succeeds
- [x] Notifications arrive in callback with real HID data
- [x] Joystick data parses correctly (X/Y values in expected range)
- [x] Deadzone eliminates drift at rest position
- [x] Tank drive math produces correct left/right motor commands
- [x] Motors auto-enable on first stick movement
- [x] Motors auto-disable on controller disconnect
- [ ] Physical verification: motors spin when joystick moved
- [x] Manual Serial commands still work (`MOTOR STATUS`, `MOTOR SET`)

## Open Questions

### Future Enhancements
1. **Button mapping:** Map A/B/X/Y buttons to mode changes (autonomous mode, emergency stop, etc.)
2. **Rate limiting:** Limit motor updates to 40ms (25Hz) to reduce UART traffic
3. **Timeout behavior:** Auto-disable motors if no HID packets received for 1 second (safety)
4. **Auto-calibration:** Measure joystick center on first connect instead of hardcoded values
5. **Multi-controller support:** Add PS4 DualShock and 8BitDo controller compatibility

### Outstanding Issues
1. **Calibration variability:** Current calibration specific to one Xbox controller unit. Do other controllers need different centers?
2. **Reconnection reliability:** NimBLE bonding keys stored, but reconnection not tested after power cycle.
3. **Concurrent connections:** Can Serial debug commands and BLE control work simultaneously? (Currently yes, but not stress-tested)

## References

- NimBLE-Arduino documentation: https://github.com/h2zero/NimBLE-Arduino
- BLE HID Service spec: https://www.bluetooth.com/specifications/specs/hid-service-1-0/
- Xbox Wireless Controller HID format: Reverse-engineered from packet captures
- Motor driver API: `motor_driver.h` in AutoBalancingAIBot/ESP32/src/
