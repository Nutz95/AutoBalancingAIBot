// pid_controller.h
#pragma once

#include <stdint.h>

namespace abbot {
namespace balancer {

class PIDController {
public:
  PIDController();
  void begin(float kp, float ki, float kd, float i_limit = 1.0f);
  void reset();
  // Compute control output given error (setpoint - measurement) and dt
  // (seconds)
  float update(float error, float error_dot, float dt);
  void setGains(float kp, float ki, float kd);

private:
  float m_kp;
  float m_ki;
  float m_kd;
  float m_integrator;
  float m_i_limit;
  float m_last_error;
};

} // namespace balancer
} // namespace abbot
