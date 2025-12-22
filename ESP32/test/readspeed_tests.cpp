// readspeed_tests.cpp
#include <iostream>
#include <cmath>
#include <cstdint>

// Small, host-only copy of the IIR speed estimator used in drivers.
class SpeedEstimator {
public:
  SpeedEstimator() : m_last_ts(0), m_last_count(0), m_speed(0.0f) {}
  float update(int64_t curCount, uint32_t now_us) {
    if (m_last_ts == 0) {
      m_last_ts = now_us;
      m_last_count = curCount;
      m_speed = 0.0f;
      return m_speed;
    }
    float dt = (now_us - m_last_ts) * 1e-6f;
    if (dt <= 0.0f) return m_speed;
    float newEst = (float)(curCount - m_last_count) / dt;
    m_speed = ALPHA * newEst + (1.0f - ALPHA) * m_speed;
    m_last_count = curCount;
    m_last_ts = now_us;
    return m_speed;
  }

private:
  uint32_t m_last_ts;
  int64_t m_last_count;
  float m_speed;
  static constexpr float ALPHA = 0.25f;
};

static int assert_approx(double a, double b, double tol=1e-2) {
  if (std::fabs(a-b) <= tol) return 0;
  std::cerr << "Assertion failed: expected " << b << " got " << a << "\n";
  return 1;
}

int main() {
  int failures = 0;
  SpeedEstimator est;

  int64_t count = 0;
  // Start time non-zero to avoid double-initialization when the first
  // sample has timestamp 0 (host test harness). Use 100ms as t0.
  uint32_t t = 100000; // microseconds

  // Initial call should return zero
  failures += assert_approx(est.update(count, t), 0.0);

  // Simulate a constant rate: 100 counts per 100000 us => 100/0.1s = 1000 counts/sec
  t += 100000; count += 100;
  double v1 = est.update(count, t);
  // After first update expect alpha * 1000 = 250
  failures += assert_approx(v1, 250.0);

  // Run several more steps to converge toward 1000
  for (int i = 0; i < 20; ++i) {
    t += 100000; count += 100;
    double v = est.update(count, t);
    // Expect monotonic approach to ~1000
    if (v > 1005.0 || v < 0.0) {
      std::cerr << "Out of range speed: " << v << "\n";
      ++failures;
      break;
    }
  }

  // Final estimate should be close to 1000
  double final = est.update(count + 100, t + 100000);
  failures += assert_approx(final, 1000.0, 10.0); // allow 10 counts/sec tolerance

  if (failures == 0) std::cout << "readSpeed tests: PASS\n";
  else std::cerr << "readSpeed tests: FAIL (" << failures << " failures)\n";
  return failures;
}
