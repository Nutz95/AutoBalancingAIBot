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
  void resetIntegrator();
  // Compute control output given error (measurement - setpoint) and dt
  // (seconds)
  float update(float error, float error_dot, float dt);
  void setGains(float kp, float ki, float kd);
  void setILimit(float limit);
  void setLeakCoeff(float leak) { m_leak_coeff = leak; }

  float getKp() const { return m_kp; }
  float getKi() const { return m_ki; }
  float getKd() const { return m_kd; }
  float getIntegrator() const { return m_integrator; }
  float getILimit() const { return m_i_limit; }
  float getLeakCoeff() const { return m_leak_coeff; }

private:
  float m_kp;
  float m_ki;
  float m_kd;
  float m_integrator;
  float m_i_limit;
  float m_leak_coeff;
  float m_last_error;
};

} // namespace balancer
} // namespace abbot
