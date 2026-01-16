#include "autotune_controller.h"
#include "logging.h"
#include <cstring>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

void AutotuneController::start(const Config *config) {
  reset();
  if (config) {
    m_config = *config;
  } else {
    m_config = Config(); // Use defaults
  }
  m_state = State::WAITING_START;
  m_elapsed_sec = 0.0f;
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
  m_elapsed_sec = 0.0f;
  memset(&m_data, 0, sizeof(m_data));
  m_data.peak_increasing = true; 
  memset(&m_result, 0, sizeof(m_result));
}

void AutotuneController::fail(const char *reason) {
  m_state = State::FAILED;
  m_result.success = false;
  m_result.failure_reason = reason;
  m_current_output = 0.0f;
}

void AutotuneController::complete() {
  m_state = State::COMPLETE;
  m_result.success = true;
  m_current_output = 0.0f;

  char msg[256];
  snprintf(msg, sizeof(msg),
           "AUTOTUNE: RESULTS - Ku=%.3f Tu=%.3fs Amp=%.2fdeg | PROPOSED: Kp=%.5f Ki=%.5f Kd=%.5f",
           (double)m_result.ultimate_gain, (double)(m_result.ultimate_period_ms / 1000.0f),
           (double)m_result.oscillation_amplitude, (double)m_result.Kp, (double)m_result.Ki, (double)m_result.Kd);
  LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, msg);
}

float AutotuneController::update(float pitch_deg, float dt_sec) {
  if (!isActive()) {
    return 0.0f;
  }

  m_elapsed_sec += dt_sec;

  // Helper: compute relay output with deadband (NO RAMP for immediate reaction)
  auto computeRelayOutput = [&](float angle_deg) {
    float abs_ang = fabsf(angle_deg);
    // Deadband: no output when within threshold
    if (abs_ang <= m_config.deadband) {
      return 0.0f;
    }
    // For a balancer: if pitch > 0 (leaning forward), wheels must move FORWARD (+amp)
    float amp = m_config.relay_amplitude;
    return (angle_deg > 0.0f) ? amp : -amp;
  };

  // Safety check: abort if pitch too large
  if (fabsf(pitch_deg) > m_config.max_pitch_abort) {
    char fail_msg[64];
    snprintf(fail_msg, sizeof(fail_msg), "Pitch exceeded safety limit (%.1f > %.1f)", 
             (double)fabsf(pitch_deg), (double)m_config.max_pitch_abort);
    fail(fail_msg);
    return 0.0f;
  }

  // Timeout check
  if (m_elapsed_sec > (float)m_config.timeout_ms / 1000.0f) {
    fail("Timeout - no oscillation detected");
    return 0.0f;
  }

  // State machine
  switch (m_state) {
  case State::WAITING_START:
    // Apply relay and wait for first zero crossing
    m_current_output = computeRelayOutput(pitch_deg);

    if (detectZeroCrossing(pitch_deg)) {
      m_state = State::COLLECTING;
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "AUTOTUNE: First zero-crossing detected, collecting cycles...");
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
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "AUTOTUNE: Data collection complete. Analyzing...");
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
  // Use a small hysteresis to avoid noise triggers near zero
  bool crossed = false;

  if (m_data.last_pitch > m_config.hysteresis && pitch_deg < -m_config.hysteresis) {
      crossed = true;
  } else if (m_data.last_pitch < -m_config.hysteresis && pitch_deg > m_config.hysteresis) {
      crossed = true;
  }

  if (crossed && m_data.num_crossings < MAX_SAMPLES) {
    m_data.crossing_times_sec[m_data.num_crossings] = m_elapsed_sec;
    m_data.num_crossings++;
    
    char msg[64];
    snprintf(msg, sizeof(msg), "AUTOTUNE: Crossing %d/%d recorded at %.2fs", 
             m_data.num_crossings, m_config.min_cycles * 2, (double)m_elapsed_sec);
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, msg);
  }

  m_data.last_pitch = pitch_deg;

  return crossed;
}

void AutotuneController::recordPeak(float pitch_deg) {
  float current_abs = fabsf(pitch_deg);

  // Peak detection logic: looking for the local maximum of the absolute pitch
  if (m_data.peak_increasing && current_abs < (m_data.last_abs_pitch - m_config.hysteresis)) {
    // Local peak confirmed by a drop greater than hysteresis
    if (m_data.num_peaks < MAX_SAMPLES && m_data.peak_candidate > (m_config.deadband * 2.0f)) {
      m_data.peak_values[m_data.num_peaks] = m_data.peak_candidate;
      
      char msg[64];
      snprintf(msg, sizeof(msg), "AUTOTUNE: Peak %d/%d detected: %.2f deg", 
               m_data.num_peaks + 1, m_config.min_cycles, (double)m_data.peak_candidate);
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, msg);
      
      m_data.num_peaks++;
    }
    m_data.peak_increasing = false;
  } else if (!m_data.peak_increasing && current_abs > (m_data.last_abs_pitch + m_config.hysteresis)) {
    // Signal is increasing again
    m_data.peak_increasing = true;
    m_data.peak_candidate = 0.0f; 
  }

  if (current_abs > m_data.peak_candidate) {
    m_data.peak_candidate = current_abs;
  }

  m_data.last_abs_pitch = current_abs;
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
  float Tu_sec = computeAveragePeriod();
  if (Tu_sec <= 0.0f) {
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
  // Kp = Ku / 2.2 (or 0.45*Ku)
  // Ti = 2.2 * Tu
  // Td = Tu / 6.3
  float Kp = 0.45f * Ku;
  float Ti = 2.2f * Tu_sec;
  float Ki = Kp / Ti; // Calculate integral term
  float Kd = Kp * Tu_sec / 6.3f;

  // Store results
  m_result.Kp = Kp;
  m_result.Ki = Ki;
  m_result.Kd = Kd;
  m_result.ultimate_gain = Ku;
  m_result.ultimate_period_ms = Tu_sec * 1000.0f;
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
    float period =
        m_data.crossing_times_sec[i] - m_data.crossing_times_sec[i - 1];
    sum += period * 2.0f; // Full cycle is 2x crossing interval
    count++;
  }

  return (count > 0) ? (sum / count) : 0.0f;
}

float AutotuneController::computeAverageAmplitude() const {
  if (m_data.num_peaks < 1) {
    return 0.0f;
  }

  // To improve stability, we ignore the first 2 peaks if we have enough data,
  // as the initial oscillations are often much larger or smaller than the steady state.
  uint8_t start_idx = (m_data.num_peaks > 4) ? 2 : 0;
  float sum = 0.0f;
  uint8_t count = 0;
  
  for (uint8_t i = start_idx; i < m_data.num_peaks; i++) {
    sum += m_data.peak_values[i];
    count++;
  }

  return (count > 0) ? (sum / count) : 0.0f;
}
