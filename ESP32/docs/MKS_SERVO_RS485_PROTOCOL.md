# MKS Servo 42D/57D RS485 Protocol Summary

This document summarizes the RS485 communication protocol for MKS Servo drivers (vFOC versions).

## Physical Interface
- **Baudrate**: Default 38400 (We use 256000)
- **Format**: 8N1 (8 data bits, no parity, 1 stop bit)
- **Wiring**: Half-duplex RS485 (requires transceiver or direct TTL-level cross-wire if using logic-level compatible versions).

## Frame Structure
### Request (from Master)
`0xFA` | `ID` | `Function` | `Data...` | `Checksum`
- `Checksum` = sum of all previous bytes, truncated to 8 bits.

### Response (ACK/Data)
Standard ACK (Functions >= 0x80):
`0xFB` | `ID` | `Function` | `Status/Data` | `Checksum`

Telemetry (Functions < 0x80):
`ID` | `Function` | `Data...` | `Checksum`

Automatic Upload (Command `0x01`):
`0xFB` | `ID` | `0x01` | `Code` | `Status` | `Checksum`

## Key Functions

### 0x01: Automatic Read-Only Parameter Upload
- **Request**: `0xFA` | `ID` | `0x01` | `Code` | `Time_H` | `Time_L` | `CS`
- **Response**: `0xFB` | `ID` | `0x01` | `Code` | `Status` | `CS`
    - `Status`: `0x01` success, `0x00` fail
- **Periodic Data Frame** (once enabled):
    - `0xFB` | `ID` | `Code` | `Param...` | `CS`
    - Example for `0x31`: `Param` = `Sign` `P3` `P2` `P1` `P0` `V_H` `V_L`
- **Disable**: set `Time_H=0` and `Time_L=0`.

### 0x46: Write All Configuration Parameters
- **Request**: `0xFA` | `ID` | `0x46` | `Params[34]` | `CS`
- **Response**: `0xFB` | `ID` | `0x46` | `Status` | `CS`
    - `Status`: `0x01` success, `0x00` fail
- **Params (Bytes 4-37)**
    1. Work mode (`0x82`)
    2. Operating current (`0x83`) (device-specific defaults)
    3. Maintaining current (`0x9B`)
    4. Job breakdown / microstep (`0x84`)
    5. Enable effective level (`0x85`)
    6. Motor direction (`0x86`)
    7. Automatic screen off (`0x87`)
    8. Stall protection (`0x88`)
    9. Subdivision interpolation (`0x89`)
    10. Baud rate (`0x8A`)
    11. Slave address (`0x8B`)
    12. Group address (`0x8D`)
    13. Response method (`0x8C`)
    14. Reserved (Modbus option `0x8E`, not used)
    15. Button lock (`0x8F`)
    16. Limit trigger level
    17. Limit direction
    18. Limit speed (`0x90`) (2 bytes)
    19. Limit enable
    20. Return distance (2 bytes)
    21. Parameter `0x94`
    22. Zero mode
    23. Zero-return current (2 bytes)
    24. Limit remapping (`0x9E`)
    25. Single zero mode
    26. Setting 0 point (`0x9A`)
    27. Return to zero speed
    28. Return to zero direction

### 0x47: Read All Configuration Parameters
- **Request**: `0xFA` | `ID` | `0x47` | `CS`
- **Response**: `0xFB` | `ID` | `0x47` | `Params[34]` | `CS`

### 0x48: Read All Status Parameters
- **Request**: `0xFA` | `ID` | `0x48` | `CS`
- **Response**: `0xFB` | `ID` | `0x48` | `Params[28]` | `CS`
- **Params (Bytes 4-31)**
    - Motor operating status (`0xF1`)
    - Encoder value (`0x31`)
    - Real-time rotation speed (`0x32`)
    - Pulse count (`0x33`)
    - IO status (`0x34`)
    - Raw encoder value (`0x35`)
    - Angular error (`0x39`)
    - Enable state (`0x3A`)
    - Single lap return to zero (`0x3B`)
    - Stalled state (`0x3E`)
    - Return to zero state (`0x3B`)

### 0xF3: Torque Enable
- **Request**: `0xFA` | `ID` | `0xF3` | `Enable (0=OFF, 1=ON)` | `CS`
- **Response**: `0xFB` | `ID` | `0xF3` | `0x01 (Success) / 0x00 (Fail)` | `CS`
- *Note*: In `CR_vFOC` (Mode 2), this command might be rejected. Switch to `SR_vFOC` (Mode 5) briefly to enable torque.

### 0x31: Read Position and Speed
- **Request**: `0xFA` | `ID` | `0x31` | `CS`
- **Response** (10 bytes): `ID` | `0x31` | `Sign (0=+, 1=-)` | `P3` | `P2` | `P1` | `P0` | `V_H` | `V_L` | `CS`
- Position = `(P3<<24 | P2<<16 | P1<<8 | P0)`
- Speed = `(V_H<<8 | V_L)`

### 0x82: Set Operating Mode
- **Request**: `0xFA` | `ID` | `0x82` | `Mode` | `CS`
- **Modes**:
    - `0`: CR_OPEN (Open loop current)
    - `1`: CR_CLOSE (Closed loop current)
    - `2`: **CR_vFOC** (Vector FOC - Pulse control)
    - `3`: SR_OPEN
    - `4`: SR_CLOSE
    - `5`: **SR_vFOC** (Vector FOC - Serial speed control)

## Hybrid Mode Logic
To achieve 1000Hz control:
1. Initialize in `CR_vFOC` (Mode 2).
2. Use ESP32 LEDC (PWM) to generate hardware pulses for speed (0 latency).
3. Start periodic telemetry using `0x01` with `Code=0x31` for encoder + speed feedback.
4. Parse the periodic telemetry in a lightweight background task (no polling loop).
5. **Mandatory**: Use physical `EN` pins for torque control. In `CR_vFOC`, serial torque commands (0xF3) are often ignored by the internal logic of the driver which prioritizes the physical state of the EN input.

## Notes
- This firmware uses the proprietary MKS protocol frames. Modbus-RTU is not used.

## Troubleshooting
- **Encoder read TIMEOUT**: Usually caused by `ENABLE_RX_ECHO` on the MKS servo (transmitting back what it receives). Firmware must skip the request echo before reading the response.
- **Command Error (0x00)**: Often occurs when trying to enable torque while in Pulse mode (Mode 2) without the physical EN pin pulled low.
