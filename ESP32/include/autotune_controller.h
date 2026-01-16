#pragma once

#include <cmath>
#include <cstdint>

/**
 * @brief PID Autotuner using Relay Feedback Method (Åström-Hägglund)
 *
 * Applies relay control (bang-bang) to induce sustained oscillations,
 * then calculates optimal PID gains based on oscillation characteristics.
 *
 * Theory:
 * - Ultimate gain: Ku = (4 * relay_amplitude) / (π * oscillation_amplitude)
 * - Ultimate period: Tu = average time between zero crossings
 * - Tyreus-Luyben tuning: Kp = 0.45*Ku, Ti = 2.2*Tu, Td = Tu/6.3
 */
class AutotuneController {
public:
  static constexpr uint8_t MAX_SAMPLES = 32;

  enum class State {
    IDLE,          ///< Not running
    WAITING_START, ///< Waiting for first zero crossing
    COLLECTING,    ///< Recording oscillation data
    ANALYZING,     ///< Computing gains
    COMPLETE,      ///< Tuning successful
    FAILED         ///< Tuning failed (timeout, instability, etc.)
  };

  struct Config {
    float relay_amplitude =
        0.25f; ///< Relay output magnitude (±) - Reduced for safety during tuning
    float deadband = 0.05f; ///< Deadband around zero (degrees) - Reduced to
                            ///< minimize coasting
    float hysteresis = 0.02f; ///< Noise filter for zero-crossing (degrees)
    float max_pitch_abort =
        35.0f;                  ///< Abort if pitch exceeds this (safety limit)
    uint32_t timeout_ms = 15000; ///< Max time to wait for oscillations
    uint32_t min_cycles = 4;    ///< Minimum consistent cycles required
  };

  struct Result {
    bool success = false;
    float Kp = 0.0f;
    float Ki = 0.0f;
    float Kd = 0.0f;
    float ultimate_gain = 0.0f;         ///< Ku (for analysis)
    float ultimate_period_ms = 0.0f;    ///< Tu (for analysis)
    float oscillation_amplitude = 0.0f; ///< Peak-to-peak pitch
    const char *failure_reason = nullptr;
  };

  AutotuneController() = default;

  /**
   * @brief Start autotuning process
   * @param config Tuning parameters (use default if not specified)
   */
  void start(const Config *config = nullptr);

  /**
   * @brief Stop autotuning and return to idle
   */
  void stop();

  /**
   * @brief Update autotune state machine (call every control cycle)
   * @param pitch_deg Current pitch angle in degrees
   * @param dt_sec Time since last update in seconds
   * @return Relay command output (or 0.0 if not running)
   */
  float update(float pitch_deg, float dt_sec);

  /**
   * @brief Get current tuning state
   */
  State getState() const { return m_state; }

  /**
   * @brief Get tuning result (valid when state is COMPLETE)
   */
  const Result &getResult() const { return m_result; }

  /**
   * @brief Get failure reason (valid when state is FAILED)
   */
  const char *getFailureReason() const { return m_result.failure_reason; }

  /**
   * @brief Check if autotuning is active
   */
  bool isActive() const {
    return m_state != State::IDLE && m_state != State::COMPLETE &&
           m_state != State::FAILED;
  }

private:
  struct OscillationData {
    float crossing_times_sec[MAX_SAMPLES] = {0}; ///< Timestamps of zero crossings
    float peak_values[MAX_SAMPLES] = {0.0f};       ///< Peak pitch values
    uint8_t num_crossings = 0;
    uint8_t num_peaks = 0;
    float last_pitch = 0.0f;
    bool peak_increasing = true;
    float peak_candidate = 0.0f;
    float last_abs_pitch = 0.0f;
  };

  void reset();
  void fail(const char *reason);
  void complete();
  bool detectZeroCrossing(float pitch_deg);
  void recordPeak(float pitch_deg);
  bool hasEnoughData() const;
  void computeGains();
  float computeAveragePeriod() const;
  float computeAverageAmplitude() const;

  State m_state = State::IDLE;
  Config m_config;
  Result m_result;
  OscillationData m_data;
  float m_elapsed_sec = 0.0f;
  float m_current_output = 0.0f;
};
