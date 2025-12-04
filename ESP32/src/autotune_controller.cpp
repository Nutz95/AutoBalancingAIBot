#include "autotune_controller.h"
#include <cstring>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

void AutotuneController::start(const Config* config) {
    reset();
    if (config) {
        m_config = *config;
    } else {
        m_config = Config(); // Use defaults
    }
    m_state = State::WAITING_START;
    m_start_time_ms = 0;
    m_elapsed_ms = 0;
}

void AutotuneController::stop() {
    if (isActive()) {
        fail("Manual stop");
    }
    reset();
}

void AutotuneController::reset() {
    m_state = State::IDLE;
    m_current_output = 0.0f;
    m_start_time_ms = 0;
    m_elapsed_ms = 0;
    memset(&m_data, 0, sizeof(m_data));
    memset(&m_result, 0, sizeof(m_result));
}

void AutotuneController::fail(const char* reason) {
    m_state = State::FAILED;
    m_result.success = false;
    m_result.failure_reason = reason;
    m_current_output = 0.0f;
}

void AutotuneController::complete() {
    m_state = State::COMPLETE;
    m_result.success = true;
    m_current_output = 0.0f;
}

float AutotuneController::update(float pitch_deg, uint32_t dt_ms) {
    if (!isActive()) {
        return 0.0f;
    }

    // Initialize start time on first update
    if (m_start_time_ms == 0) {
        m_start_time_ms = dt_ms;
    }

    m_elapsed_ms += dt_ms;

    // Safety check: abort if pitch too large
    if (fabsf(pitch_deg) > m_config.max_pitch_abort) {
        fail("Pitch exceeded safety limit");
        return 0.0f;
    }

    // Timeout check
    if (m_elapsed_ms > m_config.timeout_ms) {
        fail("Timeout - no oscillation detected");
        return 0.0f;
    }

    // Helper: compute relay output with deadband (NO RAMP for immediate reaction)
    auto computeRelayOutput = [&](float angle_deg) {
        float abs_ang = fabsf(angle_deg);
        // Deadband: no output when within threshold
        if (abs_ang <= m_config.deadband) {
            return 0.0f;
        }
        // Immediate full amplitude
        float amp = m_config.relay_amplitude;
        return (angle_deg > 0.0f) ? -amp : amp;
    };

    // State machine
    switch (m_state) {
        case State::WAITING_START:
            // Apply relay (with deadband + ramp) and wait for first zero crossing
            m_current_output = computeRelayOutput(pitch_deg);
            
            if (detectZeroCrossing(pitch_deg)) {
                m_state = State::COLLECTING;
            }
            break;

        case State::COLLECTING:
            // Continue relay control and record oscillation data
            m_current_output = computeRelayOutput(pitch_deg);

            detectZeroCrossing(pitch_deg);
            recordPeak(pitch_deg);

            // Check if we have enough data
            if (hasEnoughData()) {
                m_state = State::ANALYZING;
            }
            break;

        case State::ANALYZING:
            computeGains();
            complete();
            break;

        default:
            m_current_output = 0.0f;
            break;
    }

    return m_current_output;
}

bool AutotuneController::detectZeroCrossing(float pitch_deg) {
    bool above_zero = pitch_deg > 0.0f;
    
    // Simple zero crossing: sign change
    bool crossed = false;
    if (m_data.last_pitch != 0.0f) {
        // Check if sign changed
        bool last_above = m_data.last_pitch > 0.0f;
        if (last_above != above_zero) {
            crossed = true;
        }
    }

    if (crossed && m_data.num_crossings < 10) {
        m_data.crossing_times_ms[m_data.num_crossings] = m_elapsed_ms;
        m_data.num_crossings++;
    }

    m_data.last_pitch = pitch_deg;
    
    return crossed;
}

void AutotuneController::recordPeak(float pitch_deg) {
    // Simple peak detection: record if magnitude increased then decreased
    static float last_abs = 0.0f;
    static float peak_candidate = 0.0f;
    static bool increasing = true;

    float current_abs = fabsf(pitch_deg);

    if (increasing && current_abs < last_abs) {
        // Peak detected
        if (m_data.num_peaks < 10 && peak_candidate > m_config.deadband) {
            m_data.peak_values[m_data.num_peaks] = peak_candidate;
            m_data.num_peaks++;
        }
        increasing = false;
    } else if (!increasing && current_abs > last_abs) {
        increasing = true;
    }

    if (current_abs > peak_candidate) {
        peak_candidate = current_abs;
    }

    last_abs = current_abs;
}

bool AutotuneController::hasEnoughData() const {
    // Need at least min_cycles * 2 crossings (each cycle has 2 crossings)
    return m_data.num_crossings >= (m_config.min_cycles * 2) &&
           m_data.num_peaks >= m_config.min_cycles;
}

void AutotuneController::computeGains() {
    if (m_data.num_crossings < 2 || m_data.num_peaks < 1) {
        fail("Insufficient oscillation data");
        return;
    }

    // Calculate ultimate period (average time between crossings)
    float Tu_ms = computeAveragePeriod();
    if (Tu_ms <= 0.0f) {
        fail("Invalid oscillation period");
        return;
    }

    // Calculate oscillation amplitude (peak, zero-to-peak)
    float amplitude = computeAverageAmplitude();
    if (amplitude <= 0.0f) {
        fail("Invalid oscillation amplitude");
        return;
    }

    // Calculate ultimate gain: Ku = (4 * d) / (π * a)
    // where d = relay amplitude, a = oscillation amplitude (peak)
    float Ku = (4.0f * m_config.relay_amplitude) / (M_PI * amplitude);

    // Tyreus-Luyben tuning rules (more conservative than Ziegler-Nichols)
    // Kp = 0.45 * Ku
    // Ti = 2.2 * Tu  =>  Ki = Kp / Ti = 0.45*Ku / (2.2*Tu)
    // Td = Tu / 6.3  =>  Kd = Kp * Td = 0.45*Ku * Tu/6.3
    float Tu_sec = Tu_ms / 1000.0f;
    float Kp = 0.45f * Ku;
    float Ki = 0.0f; // Start with no integral term
    float Kd = Kp * Tu_sec / 6.3f;

    // Store results
    m_result.Kp = Kp;
    m_result.Ki = Ki;
    m_result.Kd = Kd;
    m_result.ultimate_gain = Ku;
    m_result.ultimate_period_ms = Tu_ms;
    m_result.oscillation_amplitude = amplitude;
    m_result.success = true;
}

float AutotuneController::computeAveragePeriod() const {
    if (m_data.num_crossings < 2) {
        return 0.0f;
    }

    // Average the periods between consecutive crossings
    // Period is 2x the time between crossings (each cycle has 2 crossings)
    float sum = 0.0f;
    uint8_t count = 0;

    for (uint8_t i = 1; i < m_data.num_crossings; i++) {
        uint32_t period = m_data.crossing_times_ms[i] - m_data.crossing_times_ms[i-1];
        sum += period * 2.0f; // Full cycle is 2x crossing interval
        count++;
    }

    return (count > 0) ? (sum / count) : 0.0f;
}

float AutotuneController::computeAverageAmplitude() const {
    if (m_data.num_peaks < 1) {
        return 0.0f;
    }

    // Average the peak amplitudes (zero-to-peak)
    float sum = 0.0f;
    for (uint8_t i = 0; i < m_data.num_peaks; i++) {
        sum += m_data.peak_values[i];
    }

    return sum / m_data.num_peaks;
}
