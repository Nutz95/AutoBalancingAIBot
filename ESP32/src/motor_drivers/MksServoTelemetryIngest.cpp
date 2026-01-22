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

    // Recover the full frame for the parser
    uint8_t full_frame[16];
    full_frame[0] = 0xFB;
    full_frame[1] = frame.address;
    full_frame[2] = frame.function_code;
    for (size_t i = 0; i < frame.payload_length && i < 12; ++i) {
        full_frame[3 + i] = frame.payload[i];
    }
    
    if (frame.function_code == MksServoProtocol::FUNC_READ_TELEMETRY) { // 0x31
        int32_t position;
        int16_t speed_dummy;
        if (MksServoProtocol::parseTelemetryPayload(full_frame, position, speed_dummy)) {
            if (m_invertFlag != nullptr && *m_invertFlag) {
                position = -position;
            }
            m_lastPosition = position;
            m_lastPositionTimeUs = (uint32_t)esp_timer_get_time();
            
            sample_out.position = position;
            // Internal estimator for speed (fallback or smoothed)
            sample_out.speed = m_speedEstimator.update(position, esp_timer_get_time());
            return true;
        }
    } else if (frame.function_code == MksServoProtocol::FUNC_READ_SPEED) { // 0x32
        int16_t rpm;
        if (MksServoProtocol::parseSpeedPayload(full_frame, rpm)) {
            // Convert RPM to ticks/sec
            // 16384 ticks per rev.
            // ticks_per_sec = (RPM * 16384) / 60
            float ticks_per_sec = ((float)rpm * 16384.0f) / 60.0f;
            if (m_invertFlag != nullptr && *m_invertFlag) {
                ticks_per_sec = -ticks_per_sec;
            }
            sample_out.position = m_lastPosition;
            sample_out.speed = ticks_per_sec;
            sample_out.raw_speed = ticks_per_sec;
            return true;
        }
    }

    return false;
}

void MksServoTelemetryIngest::reset() {
    m_speedEstimator.reset();
    m_lastPosition = 0;
    m_lastPositionTimeUs = 0;
    m_last_log_ms = 0;
}

} // namespace motor
} // namespace abbot
