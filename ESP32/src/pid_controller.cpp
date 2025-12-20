#include "pid_controller.h"

namespace abbot {
namespace balancer {

PIDController::PIDController()
    : m_kp(0.0f), m_ki(0.0f), m_kd(0.0f), m_integrator(0.0f), m_i_limit(1.0f),
      m_last_error(0.0f) {}

void PIDController::begin(float kp, float ki, float kd, float i_limit) {
  m_kp = kp;
  m_ki = ki;
  m_kd = kd;
  m_i_limit = i_limit;
  m_integrator = 0.0f;
  m_last_error = 0.0f;
}

void PIDController::reset() {
  m_integrator = 0.0f;
  m_last_error = 0.0f;
}

void PIDController::setGains(float kp, float ki, float kd) {
  m_kp = kp;
  m_ki = ki;
  m_kd = kd;
}

float PIDController::update(float error, float error_dot, float dt) {
  if (dt <= 0.0f)
    return 0.0f;
  // Proportional
  float p = m_kp * error;
  // Integral (trapezoidal)
  m_integrator += 0.5f * (error + m_last_error) * dt;
  // anti-windup clamp
  if (m_integrator > m_i_limit)
    m_integrator = m_i_limit;
  if (m_integrator < -m_i_limit)
    m_integrator = -m_i_limit;
  float i = m_ki * m_integrator;
  // Derivative: use provided error_dot if available (e.g., from gyro fusion)
  float d = m_kd * error_dot;
  m_last_error = error;
  return p + i + d;
}

} // namespace balancer
} // namespace abbot
