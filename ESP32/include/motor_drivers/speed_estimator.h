// speed_estimator.h
#pragma once
#include <cstdint>

namespace abbot {
namespace motor {

/**
 * @brief Simple IIR-based speed estimator used by motor drivers.
 *
 * This single-responsibility helper computes a filtered velocity (in
 * encoder counts per second) from cumulative encoder counts and a
 * monotonic timestamp expressed in microseconds.
 *
 * Usage:
 *  - Call `update(curCount, now_us)` with the latest cumulative
 *    encoder count and a monotonic timestamp in microseconds.
 *  - `update()` returns the current filtered estimate and updates
 *    internal state. `get()` returns the last estimate without
 *    modifying state.
 *
 * Thread-safety: the class is not internally synchronized. Callers must
 * ensure single-threaded access or provide external synchronization
 * when used from multiple tasks/ISRs.
 */
class SpeedEstimator {
public:
  /**
   * @brief Construct a SpeedEstimator.
   * @param alpha IIR smoothing factor in range (0, 1]. Values closer to
   * 0 produce heavier smoothing; values closer to 1 follow measurements
   * more closely. Typical values are 0.1..0.5.
   */
  explicit SpeedEstimator(float alpha = 0.25f);
  /**
   * @brief Reset internal state; sets last timestamp/count to zero and
   * clears the current speed estimate.
   */
  void reset();
  // Update estimator with current cumulative encoder count and timestamp
  // `now_us` in microseconds. Returns the new speed estimate (counts/sec).
  float update(int64_t curCount, uint64_t now_us);
  float get() const { return m_speed; }

private:
  uint64_t m_last_ts_us;
  int64_t m_last_count;
  float m_speed;
  float m_alpha;
};

} // namespace motor
} // namespace abbot
