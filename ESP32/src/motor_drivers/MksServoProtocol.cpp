#include "motor_drivers/MksServoProtocol.h"
#include "logging.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace abbot {
namespace motor {

MksServoProtocol::MksServoProtocol(HardwareSerial &serial)
    : m_serial(serial) {
    resetParserState();
}

bool MksServoProtocol::sendSpeedCommand(uint8_t address,
                                        uint16_t speed_value,
                                        uint8_t direction,
                                        uint8_t accel,
                                        StatusCode &status_out) {
    uint8_t frame[7];
    frame[0] = 0xFA;
    frame[1] = address;
    frame[2] = 0xF6;
    frame[3] = (uint8_t)((direction << 7) | ((speed_value >> 8) & 0x0F));
    frame[4] = (uint8_t)(speed_value & 0xFF);
    frame[5] = accel;
    frame[6] = computeChecksum(frame, 6);

    return writeFrame(frame, 7, status_out);
}

bool MksServoProtocol::setMode(uint8_t address, MksServoMode mode, StatusCode &status_out) {
    uint8_t data = (uint8_t)mode;
    return sendFunctionCommand(address, 0x82, &data, 1, status_out);
}

bool MksServoProtocol::setBaudRate(uint8_t address, uint8_t baud_code, StatusCode &status_out) {
    return sendFunctionCommand(address, 0x8A, &baud_code, 1, status_out);
}

bool MksServoProtocol::setResponseMode(uint8_t address, bool enabled_respond, bool enabled_active, StatusCode &status_out) {
    uint8_t data[2];
    data[0] = enabled_respond ? 0x01 : 0x00;
    data[1] = enabled_active ? 0x01 : 0x00;
    return sendFunctionCommand(address, 0x8C, data, 2, status_out);
}

bool MksServoProtocol::setMicrostep(uint8_t address, MksServoMicrostep microstep, StatusCode &status_out) {
    uint8_t data = (uint8_t)microstep;
    return sendFunctionCommand(address, 0x84, &data, 1, status_out);
}

bool MksServoProtocol::setCurrent(uint8_t address, uint16_t current_ma, StatusCode &status_out) {
    uint16_t clamped_current = current_ma;
    if (clamped_current > 3000) {
        clamped_current = 3000;
    }

    uint8_t data[2];
    data[0] = (uint8_t)((clamped_current >> 8) & 0xFF);
    data[1] = (uint8_t)(clamped_current & 0xFF);
    return sendFunctionCommand(address, 0x83, data, 2, status_out);
}

bool MksServoProtocol::setHoldCurrent(uint8_t address, MksServoHoldCurrent hold_pct, StatusCode &status_out) {
    uint8_t data = (uint8_t)hold_pct;
    return sendFunctionCommand(address, 0x9B, &data, 1, status_out);
}

bool MksServoProtocol::setTorqueEnable(uint8_t address, bool enable, StatusCode &status_out) {
    uint8_t data = enable ? 0x01 : 0x00;
    return sendFunctionCommand(address, 0xF3, &data, 1, status_out);
}

bool MksServoProtocol::setPeriodicReadParameter(uint8_t address,
                                                uint8_t read_only_code,
                                                uint16_t interval_ms,
                                                StatusCode &status_out) {
    uint8_t data[3];
    data[0] = read_only_code;
    data[1] = (uint8_t)((interval_ms >> 8) & 0xFF);
    data[2] = (uint8_t)(interval_ms & 0xFF);
    return sendFunctionCommand(address, 0x01, data, 3, status_out);
}

bool MksServoProtocol::writeAllConfigParameters(uint8_t address,
                                                const uint8_t *params,
                                                size_t params_length,
                                                uint32_t timeout_us,
                                                StatusCode &status_out) {
    const size_t expected_param_length = 34;
    if (params_length != expected_param_length) {
        status_out = StatusCode::InvalidResponse;
        return false;
    }

    if (!sendFunctionCommand(address, 0x46, params, params_length, status_out)) {
        return false;
    }

    uint8_t response[5];
    return readResponse(address, 0x46, response, sizeof(response), timeout_us, status_out);
}

bool MksServoProtocol::readAllConfigParameters(uint8_t address,
                                               uint8_t *params_out,
                                               size_t params_length,
                                               uint32_t timeout_us,
                                               StatusCode &status_out) {
    const size_t expected_param_length = 34;
    const size_t expected_frame_length = 38;
    if (params_length < expected_param_length) {
        status_out = StatusCode::ResponseTooShort;
        return false;
    }

    if (!sendReadOnlyRequest(address, 0x47, status_out)) {
        return false;
    }

    uint8_t response[38];
    if (!readResponse(address, 0x47, response, expected_frame_length, timeout_us, status_out)) {
        return false;
    }

    for (size_t index = 0; index < expected_param_length; ++index) {
        params_out[index] = response[3 + index];
    }
    return true;
}

bool MksServoProtocol::readAllStatusParameters(uint8_t address,
                                               uint8_t *params_out,
                                               size_t params_length,
                                               uint32_t timeout_us,
                                               StatusCode &status_out) {
    const size_t expected_param_length = 28;
    const size_t expected_frame_length = 32;
    if (params_length < expected_param_length) {
        status_out = StatusCode::ResponseTooShort;
        return false;
    }

    if (!sendReadOnlyRequest(address, 0x48, status_out)) {
        return false;
    }

    uint8_t response[32];
    if (!readResponse(address, 0x48, response, expected_frame_length, timeout_us, status_out)) {
        return false;
    }

    for (size_t index = 0; index < expected_param_length; ++index) {
        params_out[index] = response[3 + index];
    }
    return true;
}

bool MksServoProtocol::sendReadOnlyRequest(uint8_t address, uint8_t function_code, StatusCode &status_out) {
    uint8_t frame[4];
    frame[0] = 0xFA;
    frame[1] = address;
    frame[2] = function_code;
    frame[3] = computeChecksum(frame, 3);

#if MKS_SERVO_LOG_HEX
    LOG_PRINTF(abbot::log::CHANNEL_MOTOR, "mks_protocol: TX -> %02X %02X %02X %02X\n", frame[0], frame[1], frame[2], frame[3]);
#endif

    return writeFrame(frame, 4, status_out);
}

bool MksServoProtocol::sendFunctionCommand(uint8_t address,
                                           uint8_t function_code,
                                           const uint8_t *payload,
                                           size_t payload_length,
                                           StatusCode &status_out) {
    uint8_t frame[16];
    if (payload_length > 12) {
        status_out = StatusCode::InvalidResponse;
        return false;
    }

    frame[0] = 0xFA;
    frame[1] = address;
    frame[2] = function_code;
    for (size_t index = 0; index < payload_length; ++index) {
        frame[3 + index] = payload[index];
    }
    size_t total_length = 3 + payload_length + 1;
    frame[total_length - 1] = computeChecksum(frame, total_length - 1);

#if MKS_SERVO_LOG_HEX
    char hex_tx[64];
    size_t p_tx = 0;
    for (size_t i = 0; i < total_length; ++i) {
        p_tx += snprintf(hex_tx + p_tx, sizeof(hex_tx) - p_tx, "%02X ", frame[i]);
    }
    LOG_PRINTF(abbot::log::CHANNEL_MOTOR, "mks_protocol: TX -> %s\n", hex_tx);
#endif

    return writeFrame(frame, total_length, status_out);
}

bool MksServoProtocol::parsePositionPayload(const uint8_t* buffer, int32_t& pos_out, uint8_t& status_out) {
    // Determine the packet type based on index 2 (Function Code)
    uint8_t func = buffer[2];
    status_out = 0; // Default success
    
    if (func == FUNC_READ_TELEMETRY) {
        // V1.0.9 format: FB ADDR 31 P5 P4 P3 P2 P1 P0 CS (10 bytes)
        // This is a 48-bit cumulative value.
        // Big Endian: P5 (MSB) ... P0 (LSB) at indices 3..8
        uint64_t val = 0;
        for (int i = 0; i < 6; ++i) {
            val = (val << 8) | buffer[3 + i];
        }
        // Sign-extend from 48-bit to 64-bit
        if (val & (1ULL << 47)) {
            val |= 0xFFFF000000000000ULL;
        }
        pos_out = (int32_t)val;
    } else if (func == FUNC_READ_PULSES) {
        // 32-bit received pulses: FB ADDR 33 P3 P2 P1 P0 CS (8 bytes)
        pos_out = ((int32_t)buffer[3] << 24) | ((int32_t)buffer[4] << 16) | 
                  ((int32_t)buffer[5] << 8) | (int32_t)buffer[6];
    } else {
        // Fallback for status-headed frames (FUNC >= 0x80)
        status_out = buffer[3];
        pos_out = 0;
    }
    
    return true;
}

bool MksServoProtocol::parseTelemetryPayload(const uint8_t* buffer, int32_t& pos_out, int16_t& speed_out) {
    // V1.0.9 (10-byte 0x31) only contains 48-bit position.
    // Speed estimation is handled by our internal estimators.
    uint8_t status;
    parsePositionPayload(buffer, pos_out, status);
    speed_out = 0;
    return true;
}

bool MksServoProtocol::parseSpeedPayload(const uint8_t* buffer, int16_t& speed_out) {
    // FB ADDR 32 V_H V_L CS
    // Indices: 0=FB, 1=ADDR, 2=32, 3=V_H, 4=V_L, 5=CS
    speed_out = (int16_t)((uint16_t)buffer[3] << 8 | (uint16_t)buffer[4]);
    return true;
}

bool MksServoProtocol::readResponse(uint8_t address,
                                    uint8_t function_code,
                                    uint8_t *buffer,
                                    size_t expected_length,
                                    uint32_t timeout_us) {
    StatusCode dummy;
    return readResponse(address, function_code, buffer, expected_length, timeout_us, dummy);
}

bool MksServoProtocol::readResponse(uint8_t address,
                                    uint8_t function_code,
                                    uint8_t *buffer,
                                    size_t expected_length,
                                    uint32_t timeout_us,
                                    StatusCode &status_out) {
    unsigned long start_us = micros();
    size_t received = 0;
    bool sync_on_fb = false;
    size_t active_expected_length = expected_length;

    while (micros() - start_us < timeout_us) {
        if (m_serial.available()) {
            uint8_t byte_value = (uint8_t)m_serial.read();

            if (received == 0) {
                if (byte_value == 0xFB) {
                    sync_on_fb = true;
                    active_expected_length = expected_length;
                    buffer[received++] = byte_value;
                } else if (byte_value == address && function_code < 0x80) {
                    sync_on_fb = false;
                    active_expected_length = expected_length - 1;
                    buffer[received++] = byte_value;
                }
                continue;
            }

            if (received < 64) {
                buffer[received++] = byte_value;
            } else {
                received = 0; continue;
            }

            if (sync_on_fb) {
                if (received == 2 && buffer[1] != address) {
                    received = 0; continue;
                }
                if (received == 3 && buffer[2] != function_code) {
                    received = 0; continue;
                }
            } else {
                if (received == 2 && buffer[1] != function_code) {
                    received = 0; continue;
                }
            }

            if (received >= active_expected_length) {
#if MKS_SERVO_LOG_HEX
                char hex_rx[128];
                size_t p_rx = 0;
                for (size_t i = 0; i < active_expected_length; ++i) {
                    p_rx += snprintf(hex_rx + p_rx, sizeof(hex_rx) - p_rx, "%02X ", buffer[i]);
                }
                LOG_PRINTF(abbot::log::CHANNEL_MOTOR, "mks_protocol: RX <- %s\n", hex_rx);
#endif
                uint8_t actual_checksum = computeChecksum(buffer, active_expected_length - 1);
                if (buffer[active_expected_length - 1] == actual_checksum) {
                    status_out = StatusCode::Ok;
                    if (!sync_on_fb) {
                        for (int k = (int)active_expected_length; k >= 1; k--) {
                            buffer[k] = buffer[k-1];
                        }
                        buffer[0] = 0xFB;
                    }
                    return true;
                }

                status_out = StatusCode::ChecksumMismatch;
                received = 0;
            }
        } else {
            taskYIELD();
        }
    }

    status_out = StatusCode::Timeout;
    return false;
}

bool MksServoProtocol::tryConsumeAckNonBlocking(uint8_t expected_address,
                                                uint8_t expected_function,
                                                AckParser &parser,
                                                uint8_t &status_out) {
    int available_bytes = m_serial.available();
    if (available_bytes <= 0) {
        return false;
    }

    int bytes_to_read = (available_bytes > 16) ? 16 : available_bytes;
    while (bytes_to_read-- > 0) {
        int value = m_serial.read();
        if (value < 0) {
            break;
        }
        uint8_t byte_read = (uint8_t)value;

        if (parser.echo_skip > 0) {
            parser.echo_skip--;
            continue;
        }

        if (parser.received == 0) {
            if (byte_read == 0xFA) {
                // Echo detected. Calculate length to skip based on function code.
                size_t request_length = 4U;
                if (expected_function >= 0x80) {
                    request_length = 5U;
                } else if (expected_function == 0x01) {
                    request_length = 7U;
                } else if (expected_function == 0x46) {
                    request_length = 38U;
                }
#if MKS_SERVO_LOG_ECHO
                LOG_PRINTF(abbot::log::CHANNEL_MOTOR, 
                           "mks_protocol: Detected echo START (0xFA) in ACK parser, skipping %u bytes\n", 
                           (unsigned int)(request_length - 1));
#endif
                parser.echo_skip = (uint8_t)(request_length - 1);
                continue;
            }

            if (byte_read == 0xFB) {
                parser.buffer[0] = byte_read;
                parser.received = 1;
            }
            continue;
        }

        if (parser.received == 1) {
            if (byte_read != expected_address) {
                parser.received = 0;
                if (byte_read == 0xFB) {
                    parser.buffer[0] = 0xFB;
                    parser.received = 1;
                }
                continue;
            }
            parser.buffer[1] = byte_read;
            parser.received = 2;
            continue;
        }

        if (parser.received == 2) {
            if (byte_read != expected_function) {
                parser.received = 0;
                if (byte_read == 0xFB) {
                    parser.buffer[0] = 0xFB;
                    parser.received = 1;
                }
                continue;
            }
            parser.buffer[2] = byte_read;
            parser.received = 3;
            continue;
        }

        parser.buffer[parser.received++] = byte_read;
        if (parser.received >= 5) {
            uint8_t expected_checksum = computeChecksum(parser.buffer, 4);
            uint8_t got_checksum = parser.buffer[4];
            if (expected_checksum == got_checksum) {
#if MKS_SERVO_LOG_HEX
                LOG_PRINTF(abbot::log::CHANNEL_MOTOR, "mks_protocol: RX(ACK) <- %02X %02X %02X %02X %02X\n",
                           parser.buffer[0], parser.buffer[1], parser.buffer[2], parser.buffer[3], parser.buffer[4]);
#endif
                status_out = parser.buffer[3];
                parser.received = 0;
                return true;
            }

            parser.received = 0;
            if (byte_read == 0xFB) {
                parser.buffer[0] = 0xFB;
                parser.received = 1;
            }
        }
    }

    return false;
}

bool MksServoProtocol::tryParsePeriodicTelemetry(uint8_t byte, TelemetryFrame &frame_out) {
    if (m_telemetry_parser_state == 0) {
        if (byte == 0xFB) {
            m_telemetry_parser_state = 1;
        }
        return false;
    }

    if (m_telemetry_parser_state == 1) {
        m_telemetry_parser_address = byte;
        m_telemetry_parser_state = 2;
        return false;
    }

    if (m_telemetry_parser_state == 2) {
        m_telemetry_parser_function = byte;
        m_telemetry_parser_payload_length = getPeriodicPayloadLength(byte);
        m_telemetry_parser_payload_received = 0;
        if (m_telemetry_parser_payload_length == 0) {
            resetParserState();
            return false;
        }
        m_telemetry_parser_state = 3;
        return false;
    }

    if (m_telemetry_parser_state == 3) {
        if (m_telemetry_parser_payload_received < sizeof(m_telemetry_parser_payload)) {
            m_telemetry_parser_payload[m_telemetry_parser_payload_received] = byte;
        }
        m_telemetry_parser_payload_received++;
        if (m_telemetry_parser_payload_received >= m_telemetry_parser_payload_length) {
            m_telemetry_parser_state = 4;
        }
        return false;
    }

    if (m_telemetry_parser_state == 4) {
        uint8_t checksum_buffer[19];
        size_t checksum_length = 0;
        checksum_buffer[checksum_length++] = 0xFB;
        checksum_buffer[checksum_length++] = m_telemetry_parser_address;
        checksum_buffer[checksum_length++] = m_telemetry_parser_function;
        for (size_t index = 0; index < m_telemetry_parser_payload_length; ++index) {
            checksum_buffer[checksum_length++] = m_telemetry_parser_payload[index];
        }

        uint8_t expected_checksum = computeChecksum(checksum_buffer, checksum_length);
        if (byte == expected_checksum) {
            frame_out.address = m_telemetry_parser_address;
            frame_out.function_code = m_telemetry_parser_function;
            frame_out.payload_length = m_telemetry_parser_payload_length;
            for (size_t index = 0; index < m_telemetry_parser_payload_length; ++index) {
                frame_out.payload[index] = m_telemetry_parser_payload[index];
            }
            resetParserState();
            return true;
        }

        resetParserState();
        return false;
    }

    resetParserState();
    return false;
}

void MksServoProtocol::resetTelemetryParser() {
    resetParserState();
}

uint8_t MksServoProtocol::computeChecksum(const uint8_t *data, size_t length) {
    uint8_t checksum = 0;
    for (size_t index = 0; index < length; ++index) {
        checksum = (uint8_t)(checksum + data[index]);
    }
    return checksum;
}

uint8_t MksServoProtocol::getExpectedResponseLength(uint8_t function_code) {
    switch (function_code) {
        case FUNC_READ_TELEMETRY:    return 10; // V1.0.9 manual: FB ADDR 31 V5-V0 CS (10 bytes) 
        case FUNC_READ_SPEED:        return 6;  // FB ADDR 32 V_H V_L CS (As seen in capture: FB 01 32 00 0D 3B)
        case FUNC_READ_PULSES:       return 8;  // FB ADDR 33 P3-0 CS (As seen in capture: FB 01 33 00 00 00 00 2F)
        case FUNC_READ_IO:           return 5;  // FB ADDR 34 DATA CS
        case FUNC_READ_ERROR:        return 6;  // FB ADDR 39 E1 E0 CS (6 bytes per user dump)
        case FUNC_READ_ENABLED:      return 5;  // FB ADDR 3A EN CS
        case 0x47:                   return 38; // FB ADDR 47 DATA[34] CS
        case 0x48:                   return 32; // FB ADDR 48 DATA[28] CS
        default:
            if (function_code >= 0x80) {
                return 5; // Config write responses are FB ADDR CODE STAT CS
            }
            return 5; // Fallback
    }
}

bool MksServoProtocol::writeFrame(const uint8_t *frame, size_t length, StatusCode &status_out) {
    size_t written = m_serial.write(frame, length);
    m_serial.flush();
    if (written != length) {
        status_out = StatusCode::WriteFailed;
        return false;
    }

    status_out = StatusCode::Ok;
    return true;
}

size_t MksServoProtocol::getPeriodicPayloadLength(uint8_t function_code) const {
    if (function_code == 0x31) {
        // Compatibility: Check if we expect 10-byte (6 payload) or 11-byte (7 payload) frame
        // User manual V1.0.9 says 10 bytes total (6 payload). 
        // Older versions used 11 bytes (7 payload: sign + pos + speed).
        // We stick to the manual's 6 unless we detect otherwise.
        return 6; 
    }
    if (function_code == 0x32) {
        return 2; // Speed (V_H, V_L)
    }
    if (function_code == 0x33) {
        return 4; // Position (P3-P0)
    }
    if (function_code == 0x34) {
        return 1; // IO Status
    }
    return 0; // Default or unknown
}

void MksServoProtocol::resetParserState() {
    m_telemetry_parser_state = 0;
    m_telemetry_parser_address = 0;
    m_telemetry_parser_function = 0;
    m_telemetry_parser_payload_length = 0;
    m_telemetry_parser_payload_received = 0;
    for (size_t index = 0; index < sizeof(m_telemetry_parser_payload); ++index) {
        m_telemetry_parser_payload[index] = 0;
    }
}

} // namespace motor
} // namespace abbot
