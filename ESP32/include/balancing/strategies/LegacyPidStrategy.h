#pragma once

#include "balancing/IBalancingStrategy.h"
#include "pid_controller.h"

namespace abbot {
namespace balancing {

class LegacyPidStrategy : public IBalancingStrategy {
public:
    LegacyPidStrategy();

    const char* getName() const override { return "LEGACY_PID"; }
    StrategyType getType() const override { return StrategyType::LEGACY_PID; }

    void init() override;
    void reset(float initial_pitch_rad) override;

    IBalancingStrategy::Result compute(float pitch_rad, float pitch_rate_rads, float yaw_rate_rads, float dt_s,
                                     int32_t enc_l_ticks, int32_t enc_r_ticks,
                                     float v_enc_ticks_s) override;

    void setDriveSetpoints(float v_norm, float w_norm) override;

    void loadConfig() override;
    void saveConfig() override;
    void resetToDefaults() override;

    void setGains(float kp, float ki, float kd);
    void getGains(float &kp, float &ki, float &kd);

private:
    abbot::balancer::PIDController pid_;
};

} // namespace balancing
} // namespace abbot
