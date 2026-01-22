#include "motor_drivers/MksServoTelemetryIngest.h"
#include "logging.h"

#include <esp_timer.h>

namespace abbot {
namespace motor {

MksServoTelemetryIngest::MksServoTelemetryIngest(MksServoProtocol &protocol,
                                                 SpeedEstimator &speedEstimator,
                                                 const bool *invert_flag)
    : m_protocol(protocol),
      m_speedEstimator(speedEstimator),
      m_invertFlag(invert_flag),
      m_last_log_ms(0) {
}

bool MksServoTelemetryIngest::ingestByte(uint8_t byte, TelemetrySample &sample_out) {
    MksServoProtocol::TelemetryFrame frame;
    if (!m_protocol.tryParsePeriodicTelemetry(byte, frame)) {
        return false;
    }

    if (frame.function_code != MksServoProtocol::FUNC_READ_TELEMETRY) {
        return false;
    }

    // Recover the full frame for the static parser (indices 0..10 for 0x31)
    uint8_t full_frame[11];
    full_frame[0] = 0xFB;
    full_frame[1] = frame.address;
    full_frame[2] = frame.function_code;
    for (size_t i = 0; i < frame.payload_length && i < 8; ++i) {
        full_frame[3 + i] = frame.payload[i];
    }
    
    int32_t position;
    int16_t speed;
    if (!MksServoProtocol::parseTelemetryPayload(full_frame, position, speed)) {
        return false;
    }

    if (m_invertFlag != nullptr && *m_invertFlag) {
        position = -position;
    }

    sample_out.position = position;
    sample_out.speed = m_speedEstimator.update(position, esp_timer_get_time());

    uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
    if (now - m_last_log_ms > MKS_SERVO_DIAG_LOG_ms) {
        LOG_PRINTF(abbot::log::CHANNEL_MOTOR, "mks_protocol: Parsed Periodic ID=0x%02X pos=%ld spd=%d\n", 
                   frame.address, (long)position, (int)speed);
        m_last_log_ms = now;
    }

    return true;
}

void MksServoTelemetryIngest::reset() {
    m_protocol.resetTelemetryParser();
}

} // namespace motor
} // namespace abbot
