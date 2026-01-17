#include "balancing/BalancingManager.h"
#include "balancing/strategies/LegacyPidStrategy.h"
#include "balancing/strategies/CascadedLqrStrategy.h"
#include "logging.h"
#include <Preferences.h>

namespace abbot {
namespace balancing {

BalancingManager::BalancingManager() {
    strategies_[static_cast<int>(StrategyType::LEGACY_PID)] = std::unique_ptr<LegacyPidStrategy>(new LegacyPidStrategy());
    strategies_[static_cast<int>(StrategyType::CASCADED_LQR)] = std::unique_ptr<CascadedLqrStrategy>(new CascadedLqrStrategy());
}

BalancingManager& BalancingManager::getInstance() {
    static BalancingManager instance;
    return instance;
}

void BalancingManager::init() {
    for (auto& s : strategies_) {
        if (s) {
            s->init();
        }
    }
    loadActiveStrategy();
}

void BalancingManager::setStrategy(StrategyType type) {
    if (type == current_type_) {
        return;
    }
    
    current_type_ = type;
    LOG_PRINTF(abbot::log::CHANNEL_DEFAULT, "STRATEGY: switched to %s\n", getActiveStrategy()->getName());
    saveActiveStrategy();
}

IBalancingStrategy* BalancingManager::getActiveStrategy() {
    return strategies_[static_cast<int>(current_type_)].get();
}

IBalancingStrategy* BalancingManager::getStrategy(StrategyType type) {
    return strategies_[static_cast<int>(type)].get();
}

float BalancingManager::compute(float pitch_rad, float pitch_rate_rads, float dt_s,
                              int32_t enc_l_ticks, int32_t enc_r_ticks,
                              float v_enc_ticks_s) {
    auto* active = getActiveStrategy();
    if (!active) {
        return 0.0f;
    }
    return active->compute(pitch_rad, pitch_rate_rads, dt_s, enc_l_ticks, enc_r_ticks, v_enc_ticks_s);
}

void BalancingManager::reset(float initial_pitch_rad) {
    auto* active = getActiveStrategy();
    if (active) {
        active->reset(initial_pitch_rad);
    }
}

void BalancingManager::setDriveSetpoints(float v_norm, float w_norm) {
    auto* active = getActiveStrategy();
    if (active) {
        active->setDriveSetpoints(v_norm, w_norm);
    }
}

void BalancingManager::saveActiveStrategy() {
    Preferences prefs;
    if (prefs.begin("bal_mgr", false)) {
        prefs.putInt("active_type", static_cast<int>(current_type_));
        prefs.end();
    }
}

void BalancingManager::loadActiveStrategy() {
    Preferences prefs;
    if (prefs.begin("bal_mgr", true)) {
        int type = prefs.getInt("active_type", static_cast<int>(StrategyType::LEGACY_PID));
        current_type_ = static_cast<StrategyType>(type);
        prefs.end();
    }
}

} // namespace balancing
} // namespace abbot
