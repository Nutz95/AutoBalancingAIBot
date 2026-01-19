#pragma once

#include "balancing/IBalancingStrategy.h"
#include <stdint.h>

namespace abbot {
namespace balancing {

/**
 * @brief Cascaded LQR-style strategy with Position and Speed feedback.
 * Includes Adaptive Trim logic from Navbot architecture.
 */
class CascadedLqrStrategy : public IBalancingStrategy {
public:
    struct Config {
        float k_pitch;
        float k_gyro;
        float k_dist;
        float k_speed;
        bool adaptive_trim_enabled;
        float adaptive_trim_alpha;
    };

    CascadedLqrStrategy();

    const char* getName() const override { return "CASCADED_LQR"; }
    StrategyType getType() const override { return StrategyType::CASCADED_LQR; }

    void init() override;
    void reset(float initial_pitch_rad) override;

    IBalancingStrategy::Result compute(float pitch_rad, float pitch_rate_rads, float yaw_rate_rads, float dt_s,
                                     int32_t enc_l_ticks, int32_t enc_r_ticks,
                                     float v_enc_ticks_s) override;

    void setDriveSetpoints(float v_norm, float w_norm) override;

    void loadConfig() override;
    void saveConfig() override;
    void resetToDefaults() override;

    // Strategy-specific configuration
    void setConfig(const Config& cfg);
    Config getConfig() const { return cfg_; }

private:
    Config cfg_;
    float pitch_trim_rad_ = 0.0f;
    float yaw_error_accum_rad_ = 0.0f;
    int32_t enc_dist_zeropoint_ = 0;
    bool needs_reset_enc_ = true;
    
    float v_target_speed_ = 0.0f;
    float w_target_yaw_req_ = 0.0f;

    float lp_pitch_rate_ = 0.0f;
    float lp_v_speed_ = 0.0f;

    void updateAdaptiveTrim(float pitch_rad, float dist_err, float dt);
};

} // namespace balancing
} // namespace abbot
