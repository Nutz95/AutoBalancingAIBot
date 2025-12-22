// speed_estimator.cpp
#include "../include/motor_drivers/speed_estimator.h"

namespace abbot {
namespace motor {

SpeedEstimator::SpeedEstimator(float alpha)
    : m_last_ts_us(0), m_last_count(0), m_speed(0.0f), m_alpha(alpha) {}

void SpeedEstimator::reset() {
  m_last_ts_us = 0;
  m_last_count = 0;
  m_speed = 0.0f;
}

float SpeedEstimator::update(int64_t curCount, uint32_t now_us) {
  if (m_last_ts_us == 0) {
    m_last_ts_us = now_us;
    m_last_count = curCount;
    m_speed = 0.0f;
    return m_speed;
  }
  float dt = (now_us - m_last_ts_us) * 1e-6f;
  if (dt <= 0.0f)
    return m_speed;
  float newEst = (float)(curCount - m_last_count) / dt;
  m_speed = m_alpha * newEst + (1.0f - m_alpha) * m_speed;
  m_last_count = curCount;
  m_last_ts_us = now_us;
  return m_speed;
}

} // namespace motor
} // namespace abbot
