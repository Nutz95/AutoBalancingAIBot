// MksServoProtocol.h - MKS SERVO42D/57D proprietary RS485 protocol utilities
#pragma once

#include <Arduino.h>
#include <cstddef>
#include <cstdint>

#include "../../config/motor_configs/mks_servo_config.h"

namespace abbot {
namespace motor {

class MksServoProtocol {
public:
    // MKS Servo Protocol Function Codes (Aligned with Manual V1.0.9)
    static constexpr uint8_t FUNC_READ_TELEMETRY    = 0x31; // Encoder (Cumulative)
    static constexpr uint8_t FUNC_READ_SPEED        = 0x32; // Real-time Speed
    static constexpr uint8_t FUNC_READ_PULSES       = 0x33; // Received Pulses
    static constexpr uint8_t FUNC_READ_IO           = 0x34; // I/O status
    static constexpr uint8_t FUNC_READ_ENCODER_RAW  = 0x35; // Internal Encoder Raw
    static constexpr uint8_t FUNC_READ_ERROR        = 0x39; // Position Error
    static constexpr uint8_t FUNC_READ_ENABLED      = 0x3A; // Enable status
    
    static constexpr uint8_t FUNC_READ_POS          = 0x31; // Alias for position
    static constexpr uint8_t FUNC_READ_POS_SPEED    = 0x31; // Alias for compatibility
    
    static constexpr uint8_t FUNC_CALIBRATE_ENCODER = 0x80;
    static constexpr uint8_t FUNC_SET_MODE          = 0x82;
    static constexpr uint8_t FUNC_SET_CURRENT       = 0x83;
    static constexpr uint8_t FUNC_SET_MSTEP         = 0x84;
    static constexpr uint8_t FUNC_SET_BAUD          = 0x8A;
    static constexpr uint8_t FUNC_SET_RESPONSE_MODE = 0x8C;
    static constexpr uint8_t FUNC_SET_HOLD_CURRENT  = 0x9B;
    static constexpr uint8_t FUNC_SET_ZERO_POS      = 0x9A; // Set current pos as 0
    
    static constexpr uint8_t FUNC_SET_PERIODIC      = 0x01;
    static constexpr uint8_t FUNC_TORQUE_ENABLE     = 0xF3;
    static constexpr uint8_t FUNC_SPEED_CONTROL     = 0xF6;

    enum class StatusCode : uint8_t {
        Ok = 0,
        Timeout,
        ChecksumMismatch,
        InvalidResponse,
        ResponseTooShort,
        WriteFailed,
        UnsupportedFunction
    };

    struct TelemetryFrame {
        uint8_t address = 0;
        uint8_t function_code = 0;
        uint8_t payload[16] = {0};
        size_t payload_length = 0;
    };

    struct AckParser {
        uint8_t buffer[5] = {0, 0, 0, 0, 0};
        uint8_t received = 0;
        uint8_t echo_skip = 0; // Bytes of echo remaining to skip
    };

    explicit MksServoProtocol(HardwareSerial &serial);

    // Command 0xF6: set speed in SR_vFOC mode.
    bool sendSpeedCommand(uint8_t address,
                          uint16_t speed_value,
                          uint8_t direction,
                          uint8_t accel,
                          StatusCode &status_out);

    // Command 0x82: set operating mode.
    bool setMode(uint8_t address, MksServoMode mode, StatusCode &status_out);

    // Command 0x8A: set baud rate.
    // baud_code: 0x01=9600, 0x02=19200, 0x03=25000, 0x04=38400, 0x05=57600, 0x06=115200, 0x07=256000
    bool setBaudRate(uint8_t address, uint8_t baud_code, StatusCode &status_out);

    // Command 0x8C: set response method.
    // enabled_respond: if true, slave will respond to commands (default true)
    // enabled_active: if true, slave will actively report some status changes (default true)
    bool setResponseMode(uint8_t address, bool enabled_respond, bool enabled_active, StatusCode &status_out);

    // Command 0x84: set microstep.
    bool setMicrostep(uint8_t address, MksServoMicrostep microstep, StatusCode &status_out);

    // Command 0x83: set running current.
    bool setCurrent(uint8_t address, uint16_t current_ma, StatusCode &status_out);

    // Command 0x9B: set holding current.
    bool setHoldCurrent(uint8_t address, MksServoHoldCurrent hold_pct, StatusCode &status_out);

    // Command 0xF3: torque enable/disable.
    bool setTorqueEnable(uint8_t address, bool enable, StatusCode &status_out);

    // Command 0x01: automatic upload of read-only parameter at a fixed interval.
    bool setPeriodicReadParameter(uint8_t address,
                                  uint8_t read_only_code,
                                  uint16_t interval_ms,
                                  StatusCode &status_out);

    // Command 0x46: write all configuration parameters in one frame (34 bytes).
    bool writeAllConfigParameters(uint8_t address,
                                  const uint8_t *params,
                                  size_t params_length,
                                  uint32_t timeout_us,
                                  StatusCode &status_out);

    // Command 0x47: read all configuration parameters in one frame (34 bytes).
    bool readAllConfigParameters(uint8_t address,
                                 uint8_t *params_out,
                                 size_t params_length,
                                 uint32_t timeout_us,
                                 StatusCode &status_out);

    // Command 0x48: read all status parameters in one frame (28 bytes).
    bool readAllStatusParameters(uint8_t address,
                                 uint8_t *params_out,
                                 size_t params_length,
                                 uint32_t timeout_us,
                                 StatusCode &status_out);

    // Generic read-only query (Functions < 0x80).
    bool sendReadOnlyRequest(uint8_t address, uint8_t function_code, StatusCode &status_out);

    // Generic function command frame.
    bool sendFunctionCommand(uint8_t address,
                             uint8_t function_code,
                             const uint8_t *payload,
                             size_t payload_length,
                             StatusCode &status_out);

    /**
     * @brief Parses a 6-byte payload response for position/telemetry (e.g. from 0x31, 0x33, 0x35).
     * Expects payload starting at buffer[3] for standard FB frames.
     * @param buffer Full response frame starting with 0xFB
     * @param pos_out Resulting 32-bit position
     * @param status_out Sign/Status bits
     * @return true if successfully parsed
     */
    static bool parsePositionPayload(const uint8_t* buffer, int32_t& pos_out, uint8_t& status_out);

    /**
     * @brief Parses a 0x31 Telemetry frame (Pos + Speed).
     * @param buffer Full response frame starting with 0xFB
     * @param pos_out Resulting 32-bit position
     * @param speed_out Resulting 16-bit speed
     * @return true if successfully parsed
     */
    static bool parseTelemetryPayload(const uint8_t* buffer, int32_t& pos_out, int16_t& speed_out);

    // Blocking response reader for fixed-size frames.
    bool readResponse(uint8_t address,
                      uint8_t function_code,
                      uint8_t *buffer,
                      size_t expected_length,
                      uint32_t timeout_us);

    bool readResponse(uint8_t address,
                      uint8_t function_code,
                      uint8_t *buffer,
                      size_t expected_length,
                      uint32_t timeout_us,
                      StatusCode &status_out);

    bool tryConsumeAckNonBlocking(uint8_t expected_address,
                                  uint8_t expected_function,
                                  AckParser &parser,
                                  uint8_t &status_out);

    bool tryParsePeriodicTelemetry(uint8_t byte, TelemetryFrame &frame_out);
    void resetTelemetryParser();

    static uint8_t computeChecksum(const uint8_t *data, size_t length);
    static uint8_t getExpectedResponseLength(uint8_t function_code);

private:
    bool writeFrame(const uint8_t *frame, size_t length, StatusCode &status_out);
    size_t getPeriodicPayloadLength(uint8_t function_code) const;
    void resetParserState();

    HardwareSerial &m_serial;

    uint8_t m_telemetry_parser_state = 0;
    uint8_t m_telemetry_parser_address = 0;
    uint8_t m_telemetry_parser_function = 0;
    uint8_t m_telemetry_parser_payload[16] = {0};
    size_t m_telemetry_parser_payload_length = 0;
    size_t m_telemetry_parser_payload_received = 0;
};

} // namespace motor
} // namespace abbot
