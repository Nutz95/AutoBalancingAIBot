#include "balancing/strategies/LegacyPidStrategy.h"
#include "../../../config/balancer_config.h"
#include "logging.h"
#include <Preferences.h>

namespace abbot {
namespace balancing {

LegacyPidStrategy::LegacyPidStrategy() {
}

void LegacyPidStrategy::init() {
    loadConfig();
}

float LegacyPidStrategy::compute(float pitch_rad, float pitch_rate_rads, float dt_s,
                               int32_t /*enc_l*/, int32_t /*enc_r*/, float /*v_enc*/) {
    // Legacy PID tuned for DEGREES
    return pid_.update(pitch_rad * (180.0f / M_PI), pitch_rate_rads * (180.0f / M_PI), dt_s);
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
        prefs.putFloat("kp", pid_.getKp());
        prefs.putFloat("ki", pid_.getKi());
        prefs.putFloat("kd", pid_.getKd());
        prefs.end();
    }
}

void LegacyPidStrategy::resetToDefaults() {
    pid_.setGains(BALANCER_DEFAULT_KP, BALANCER_DEFAULT_KI, BALANCER_DEFAULT_KD);
    saveConfig();
}

void LegacyPidStrategy::setGains(float kp, float ki, float kd) {
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
