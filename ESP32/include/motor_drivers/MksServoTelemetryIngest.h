// MksServoTelemetryIngest.h - Lightweight parser for periodic MKS telemetry
#pragma once

#include <cstdint>

#include "MksServoProtocol.h"
#include "speed_estimator.h"

namespace abbot {
namespace motor {

class MksServoTelemetryIngest {
public:
    struct TelemetrySample {
        int32_t position = 0;
        float speed = 0.0f;
    };

    MksServoTelemetryIngest(MksServoProtocol &protocol,
                            SpeedEstimator &speedEstimator,
                            const bool *invert_flag);

    bool ingestByte(uint8_t byte, TelemetrySample &sample_out);
    void reset();

private:
    MksServoProtocol &m_protocol;
    SpeedEstimator &m_speedEstimator;
    const bool *m_invertFlag = nullptr;
    uint32_t m_last_log_ms = 0;
};

} // namespace motor
} // namespace abbot
