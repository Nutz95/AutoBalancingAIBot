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
`0xFB` | `ID` | `Function` | `Data...` | `Checksum`
*Note*: Some older firmware versions might omit the `0xFB` header, but V1.0.9+ standardizes on it.

Automatic Upload (Command `0x01`):
`0xFB` | `ID` | `0x01` | `Code` | `Status` | `Checksum`

## Key Functions (Aligned with Manual V1.0.9)

### 0x01: Automatic Read-Only Parameter Upload
- **Request**: `0xFA` | `ID` | `0x01` | `Code` | `Time_H` | `Time_L` | `CS`
- **Response**: `0xFB` | `ID` | `0x01` | `Code` | `Status` | `CS`
    - `Status`: `0x01` success, `0x00` fail
- **Periodic Data Frame** (once enabled):
    - `0xFB` | `ID` | `Code` | `Param...` | `CS`
    - Example for `0x31`: `Param` = `V5` `V4` `V3` `V2` `V1` `V0` (48-bit cumulative value)
- **Disable**: set `Time_H=0` and `Time_L=0`.

### 0x31: Read Cumulative Encoder Value
- **Request**: `0xFA` | `ID` | `0x31` | `CS`
- **Response** (10 bytes): `0xFB` | `ID` | `0x31` | `V5` `V4` `V3` `V2` `V1` `V0` | `CS`
- Position = `(V5<<40 | ... | V0)` (Cumulative counts, 16384 per turn)

### 0x32: Read Real-time Speed
- **Request**: `0xFA` | `ID` | `0x32` | `CS`
- **Response** (6 bytes): `0xFB` | `ID` | `0x32` | `V_H` | `V_L` | `CS`
- Speed = `(int16_t)(V_H<<8 | V_L)` (RPM)

### 0x33: Read Received Pulses
- **Request**: `0xFA` | `ID` | `0x33` | `CS`
- **Response** (8 bytes): `0xFB` | `ID` | `0x33` | `P3` `P2` `P1` `P0` | `CS`
- Position = `(int32_t)(P3<<24 | ... | P0)` (Total step pulses received)

### 0x34: Read I/O Port Status
- **Request**: `0xFA` | `ID` | `0x34` | `CS`
- **Response** (5 bytes): `0xFB` | `ID` | `0x34` | `Status` | `CS`

### 0x39: Read Position Error
- **Request**: `0xFA` | `ID` | `0x39` | `CS`
- **Response** (Varies): Manual says 8 bytes (4-byte data), but some firmware returns 5 bytes (1-byte data).

### 0x3A: Read Enable State
- **Request**: `0xFA` | `ID` | `0x3A` | `CS`
- **Response** (5 bytes): `0xFB` | `ID` | `0x3A` | `Status (1=ON, 0=OFF)` | `CS`

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
