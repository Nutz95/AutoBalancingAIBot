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

## Key Functions

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
3. Use RS485 for 0x31 telemetry (encoder + speed feedback).
4. **Mandatory**: Use physical `EN` pins for torque control. In `CR_vFOC`, serial torque commands (0xF3) are often ignored by the internal logic of the driver which prioritizes the physical state of the EN input.

## Troubleshooting
- **Encoder read TIMEOUT**: Usually caused by `ENABLE_RX_ECHO` on the MKS servo (transmitting back what it receives). Firmware must skip the request echo before reading the response.
- **Command Error (0x00)**: Often occurs when trying to enable torque while in Pulse mode (Mode 2) without the physical EN pin pulled low.
