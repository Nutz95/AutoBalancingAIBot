#pragma once

#include "IBalancingStrategy.h"
#include <memory>
#include <vector>

namespace abbot {
namespace balancing {

/**
 * @brief Singleton manager to handle balancing strategies.
 * Responsible for strategy lifecycle, NVS management, and switching.
 */
class BalancingManager {
public:
    static BalancingManager& getInstance();

    void init();
    
    // Strategy selection
    void setStrategy(StrategyType type);
    IBalancingStrategy* getActiveStrategy();
    IBalancingStrategy* getStrategy(StrategyType type);
    StrategyType getActiveType() const { return current_type_; }

    // Unified compute call
    float compute(float pitch_rad, float pitch_rate_rads, float dt_s,
                int32_t enc_l_ticks, int32_t enc_r_ticks,
                float v_enc_ticks_s);

    void reset(float initial_pitch_rad);
    void setDriveSetpoints(float v_norm, float w_norm);

    // Persistence
    void saveActiveStrategy();
    void loadActiveStrategy();

private:
    BalancingManager();
    ~BalancingManager() = default;

    std::unique_ptr<IBalancingStrategy> strategies_[2];
    StrategyType current_type_ = StrategyType::LEGACY_PID;
};

} // namespace balancing
} // namespace abbot
