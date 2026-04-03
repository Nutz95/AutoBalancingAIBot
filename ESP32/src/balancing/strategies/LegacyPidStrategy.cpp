#include "balancing/strategies/LegacyPidStrategy.h"
#include "../../../config/balancer_config.h"
#include "common/ConfigPersistence.h"
#include "logging.h"
#include <Preferences.h>

namespace {

struct PidPreferenceValue {
    const char* key;
    float value;
    float defaultValue;
};

void savePidConfigToPreferences(Preferences& preferences, float kp, float ki, float kd) {
    const PidPreferenceValue values[] = {
        {"kp", kp, BALANCER_DEFAULT_KP},
        {"ki", ki, BALANCER_DEFAULT_KI},
        {"kd", kd, BALANCER_DEFAULT_KD},
    };

    for (const auto& value : values) {
        abbot::config::putFloatIfChanged(preferences, value.key, value.value, value.defaultValue);
    }
}

}

namespace abbot {
namespace balancing {

LegacyPidStrategy::LegacyPidStrategy() {
}

void LegacyPidStrategy::init() {
    loadConfig();
}

IBalancingStrategy::Result LegacyPidStrategy::compute(float pitch_rad, float pitch_rate_rads, float yaw_rate_rads, float dt_s,
                               int32_t /*enc_l*/, int32_t /*enc_r*/, float /*v_enc*/) {
    (void)yaw_rate_rads;
    // Legacy PID tuned for DEGREES
    float cmd = pid_.update(pitch_rad * (180.0f / M_PI), pitch_rate_rads * (180.0f / M_PI), dt_s);
    return {cmd, 0.0f, pid_.getIntegrator(), 0.0f, 0.0f, 0.0f, 0.0f};
}

void LegacyPidStrategy::reset(float initial_pitch_rad) {
    (void)initial_pitch_rad;
    pid_.reset();
}

void LegacyPidStrategy::setDriveSetpoints(float v_norm, float w_norm) {
    (void)v_norm; (void)w_norm;
}

void LegacyPidStrategy::loadConfig() {
    Preferences prefs;
    if (prefs.begin("bal_pid", true)) {
        float kp = prefs.getFloat("kp", BALANCER_DEFAULT_KP);
        float ki = prefs.getFloat("ki", BALANCER_DEFAULT_KI);
        float kd = prefs.getFloat("kd", BALANCER_DEFAULT_KD);
        pid_.setGains(kp, ki, kd);
        prefs.end();
    } else {
        pid_.setGains(BALANCER_DEFAULT_KP, BALANCER_DEFAULT_KI, BALANCER_DEFAULT_KD);
    }
}

void LegacyPidStrategy::saveConfig() {
    Preferences prefs;
    if (prefs.begin("bal_pid", false)) {
        const float kp = pid_.getKp();
        const float ki = pid_.getKi();
        const float kd = pid_.getKd();
        savePidConfigToPreferences(prefs, kp, ki, kd);
        prefs.end();
    }
}

void LegacyPidStrategy::resetToDefaults() {
    pid_.setGains(BALANCER_DEFAULT_KP, BALANCER_DEFAULT_KI, BALANCER_DEFAULT_KD);
    saveConfig();
}

void LegacyPidStrategy::setGains(float kp, float ki, float kd) {
    if (abbot::config::nearlyEqual(pid_.getKp(), kp)
        && abbot::config::nearlyEqual(pid_.getKi(), ki)
        && abbot::config::nearlyEqual(pid_.getKd(), kd)) {
        return;
    }
    pid_.setGains(kp, ki, kd);
    saveConfig();
}

void LegacyPidStrategy::getGains(float &kp, float &ki, float &kd) {
    kp = pid_.getKp();
    ki = pid_.getKi();
    kd = pid_.getKd();
}

} // namespace balancing
} // namespace abbot
