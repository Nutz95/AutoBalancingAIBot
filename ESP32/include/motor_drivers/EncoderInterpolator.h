#pragma once
#include <stdint.h>
#include <atomic>
#include <cmath>

namespace abbot {
namespace motor {

/**
 * @brief Class responsible for First-Order Hold interpolation of encoder data.
 * It predicts the current position between two telemetry updates based on speed.
 * Uses predictive error-bleeding to avoid discontinuities (jumps) in position.
 */
class EncoderInterpolator {
public:
    EncoderInterpolator(float tau_s = 0.010f) 
        : m_lastPos(0), 
          m_lastSpeed(0.0f), 
          m_lastTimeUs(0),
          m_errorOffset(0.0f),
          m_tau_s(tau_s) {}

    /**
     * @brief Set the smoothing time constant.
     */
    void setTau(float tau_s) {
        m_tau_s.store(tau_s);
    }

    /**
     * @brief Update the internal state with fresh telemetry data.
     */
    void update(int32_t position, float speed, uint32_t timestamp_us) {
        // Estimate where we should have been at this timestamp
        int32_t predicted = getInterpolated(timestamp_us, 1000000); // Allow large dt for update
        
        // Discontinuity (the 'jump' or 'poil')
        float jump = (float)(position - predicted);
        
        // Add current jump to our bleeding offset, but also account for existing decay
        // so we don't accumulate double errors.
        m_errorOffset.store(m_errorOffset.load() - jump);

        m_lastPos.store(position);
        m_lastSpeed.store(speed);
        m_lastTimeUs.store(timestamp_us);
    }

    /**
     * @brief Estimate the position at now_us.
     * @param now_us Current time in microseconds
     * @param max_extrapolate_us Max time in microseconds we are allowed to project forward.
     */
    int32_t getInterpolated(uint32_t now_us, uint32_t max_extrapolate_us) const {
        uint32_t last_us = m_lastTimeUs.load();
        int32_t base_pos = m_lastPos.load();
        
        if (last_us == 0) {
            return base_pos;
        }

        uint32_t dt_us = now_us - last_us;
        
        // Safety: Cap extrapolation to avoid runaway values if bus is lost
        if (dt_us > max_extrapolate_us) {
            return base_pos;
        }

        float speed_counts_per_s = m_lastSpeed.load();
        float dt_s = (float)dt_us * 1.0e-6f;

        // Raw extrapolation (truth trajectory)
        float raw_prediction = (float)base_pos + (speed_counts_per_s * dt_s);

        // Apply error-bleeding (nudging) to smooth jumps.
        // Tau (bleeding time) to bleed out discontinuities.
        float offset = m_errorOffset.load();
        float tau = m_tau_s.load();
        if (tau <= 0.0f) {
            return (int32_t)raw_prediction;
        }
        float decay = expf(-dt_s / tau);
        
        return (int32_t)(raw_prediction + offset * decay);
    }

    void reset() {
        m_lastPos.store(0);
        m_lastSpeed.store(0.0f);
        m_lastTimeUs.store(0);
        m_errorOffset.store(0.0f);
    }

private:
    std::atomic<int32_t> m_lastPos;
    std::atomic<float> m_lastSpeed;
    std::atomic<uint32_t> m_lastTimeUs;
    std::atomic<float> m_errorOffset;
    std::atomic<float> m_tau_s;
};

} // namespace motor
} // namespace abbot
