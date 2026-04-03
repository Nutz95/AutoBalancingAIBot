#include "balancing/strategies/CascadedLqrStrategy.h"
#include "../../../config/balancer_config.h"
#include "common/ConfigPersistence.h"
#include "logging.h"
#include <Preferences.h>
#include <algorithm>
#include <cmath>
#include <esp_attr.h>

namespace abbot {
namespace balancing {

namespace {

constexpr float kMinUsableLqrFilterHz = 0.5f;

using LqrConfiguration = CascadedLqrStrategy::Config;

struct FloatConfigFieldBinding {
    const char* key;
    float LqrConfiguration::* member;
    float defaultValue;
};

struct BoolConfigFieldBinding {
    const char* key;
    bool LqrConfiguration::* member;
    bool defaultValue;
};

constexpr FloatConfigFieldBinding FLOAT_CONFIG_FIELD_BINDINGS[] = {
    {"kp", &LqrConfiguration::k_pitch, BALANCER_DEFAULT_K_PITCH},
    {"kg", &LqrConfiguration::k_gyro, BALANCER_DEFAULT_K_GYRO},
    {"kd", &LqrConfiguration::k_dist, BALANCER_DEFAULT_K_DIST},
    {"ks", &LqrConfiguration::k_speed, BALANCER_DEFAULT_K_SPEED},
    {"ky", &LqrConfiguration::k_yaw, BALANCER_DEFAULT_K_YAW},
    {"kyr", &LqrConfiguration::k_yaw_rate, BALANCER_DEFAULT_K_YAW_RATE},
    {"ata", &LqrConfiguration::adaptive_trim_alpha, BALANCER_ADAPTIVE_TRIM_ALPHA},
    {"atdb", &LqrConfiguration::adaptive_trim_deadband, BALANCER_ADAPTIVE_TRIM_DEADBAND_TICKS},
    {"prhz", &LqrConfiguration::pitch_rate_lpf_hz, BALANCER_LQR_PITCH_RATE_LPF_HZ},
    {"cmdhz", &LqrConfiguration::cmd_lpf_hz, BALANCER_LQR_CMD_LPF_HZ},
};

constexpr BoolConfigFieldBinding BOOL_CONFIG_FIELD_BINDINGS[] = {
    {"ate", &LqrConfiguration::adaptive_trim_enabled, (BALANCER_ENABLE_ADAPTIVE_TRIM != 0)},
};

void loadConfigFromPreferences(Preferences& preferences, LqrConfiguration& configuration) {
    for (const auto& binding : FLOAT_CONFIG_FIELD_BINDINGS) {
        configuration.*(binding.member) = preferences.getFloat(binding.key, binding.defaultValue);
    }
    for (const auto& binding : BOOL_CONFIG_FIELD_BINDINGS) {
        configuration.*(binding.member) = preferences.getBool(binding.key, binding.defaultValue);
    }
}

void saveConfigToPreferences(Preferences& preferences, const LqrConfiguration& configuration) {
    for (const auto& binding : FLOAT_CONFIG_FIELD_BINDINGS) {
        abbot::config::putFloatIfChanged(
            preferences,
            binding.key,
            configuration.*(binding.member),
            binding.defaultValue);
    }
    for (const auto& binding : BOOL_CONFIG_FIELD_BINDINGS) {
        abbot::config::putBoolIfChanged(
            preferences,
            binding.key,
            configuration.*(binding.member),
            binding.defaultValue);
    }
}

bool configsEqual(const LqrConfiguration& left, const LqrConfiguration& right) {
    for (const auto& binding : FLOAT_CONFIG_FIELD_BINDINGS) {
        if (!abbot::config::nearlyEqual(left.*(binding.member), right.*(binding.member))) {
            return false;
        }
    }
    for (const auto& binding : BOOL_CONFIG_FIELD_BINDINGS) {
        if ((left.*(binding.member)) != (right.*(binding.member))) {
            return false;
        }
    }
    return true;
}

float sanitizeLqrFilterHz(float hz) {
    if (!std::isfinite(hz) || hz < 0.0f) {
        return 0.0f;
    }
    if (hz > 0.0f && hz < kMinUsableLqrFilterHz) {
        return 0.0f;
    }
    return hz;
}

void sanitizeConfigFilters(LqrConfiguration& configuration) {
    configuration.pitch_rate_lpf_hz = sanitizeLqrFilterHz(configuration.pitch_rate_lpf_hz);
    configuration.cmd_lpf_hz = sanitizeLqrFilterHz(configuration.cmd_lpf_hz);
}

} // namespace

CascadedLqrStrategy::CascadedLqrStrategy() {
    // Initialize sane in-memory defaults without touching NVS.
    // Persisted values must only be overwritten by an explicit reset
    // or an intentional configuration update.
    configuration_.k_pitch = BALANCER_DEFAULT_K_PITCH;
    configuration_.k_gyro  = BALANCER_DEFAULT_K_GYRO;
    configuration_.k_dist  = BALANCER_DEFAULT_K_DIST;
    configuration_.k_speed = BALANCER_DEFAULT_K_SPEED;
    configuration_.k_yaw = BALANCER_DEFAULT_K_YAW;
    configuration_.k_yaw_rate = BALANCER_DEFAULT_K_YAW_RATE;
    configuration_.adaptive_trim_enabled = (BALANCER_ENABLE_ADAPTIVE_TRIM != 0);
    configuration_.adaptive_trim_alpha = BALANCER_ADAPTIVE_TRIM_ALPHA;
    configuration_.adaptive_trim_deadband = BALANCER_ADAPTIVE_TRIM_DEADBAND_TICKS;
    configuration_.pitch_rate_lpf_hz = BALANCER_LQR_PITCH_RATE_LPF_HZ;
    configuration_.cmd_lpf_hz = BALANCER_LQR_CMD_LPF_HZ;

    pitch_trim_rad_ = 0.0f;
    v_target_speed_ = 0.0f;
    w_target_yaw_req_ = 0.0f;
    yaw_error_accum_rad_ = 0.0f;
    lp_pitch_rate_ = 0.0f;
    lp_v_speed_ = 0.0f;
    lp_cmd_ = 0.0f;
    enc_dist_zeropoint_ = 0;
    needs_reset_enc_ = true;
}

void CascadedLqrStrategy::init() {
    loadConfig();
}

void CascadedLqrStrategy::reset(float initial_pitch_rad) {
    pitch_trim_rad_ = initial_pitch_rad;
    v_target_speed_ = 0.0f;
    w_target_yaw_req_ = 0.0f;
    yaw_error_accum_rad_ = 0.0f;
    lp_pitch_rate_ = 0.0f;
    lp_v_speed_ = 0.0f;
    needs_reset_enc_ = true;
}

IBalancingStrategy::Result IRAM_ATTR CascadedLqrStrategy::compute(float pitch_rad, float pitch_rate_rads, float yaw_rate_rads, float dt_s,
                                 int32_t enc_l_ticks, int32_t enc_r_ticks,
                                 float v_enc_ticks_s) {
    
    // Low-pass filter the pitch rate to reduce high-frequency vibrations in motor commands.
    // Use dynamic Hz if configured, otherwise fallback to macro-defined alpha.
    float alpha_pitch = BALANCER_PITCH_RATE_LPF_ALPHA;
    if (configuration_.pitch_rate_lpf_hz > 0.001f) {
        float tau = 1.0f / (2.0f * (float)M_PI * configuration_.pitch_rate_lpf_hz);
        alpha_pitch = dt_s / (tau + dt_s);
    }
    lp_pitch_rate_ = (lp_pitch_rate_ * (1.0f - alpha_pitch)) + (pitch_rate_rads * alpha_pitch);

    // Filter noisy velocity from high-resolution encoders
    const float alpha_speed = BALANCER_SPEED_LPF_ALPHA; 
    lp_v_speed_ = (lp_v_speed_ * (1.0f - alpha_speed)) + (v_enc_ticks_s * alpha_speed);

    // 1. Position tracking (LQR state x)
    // Applying the "divide by 10" suggestion to handle high-resolution encoder noise/scaling
    int32_t avg_enc = (enc_l_ticks + enc_r_ticks) / 2;
    if (needs_reset_enc_) {
        enc_dist_zeropoint_ = avg_enc;
        needs_reset_enc_ = false;
        lp_pitch_rate_ = pitch_rate_rads;
        lp_v_speed_ = v_enc_ticks_s;
    }
    // v71 FIX: Sign check. If motors/encoders are inverted, k_dist becomes positive feedback.
    // Based on capture_15, term_dist was POSITIVE while position was POSITIVE, pushing the robot.
    float dist_err = (float)(avg_enc - enc_dist_zeropoint_) / BALANCER_ENCODER_DOWNSCALE;
    float v_enc_scaled = lp_v_speed_ / BALANCER_ENCODER_DOWNSCALE;

    // 2. Yaw Tracking (Heading Hold)
    // Integrate yaw rate to get relative heading error from start
    yaw_error_accum_rad_ += (yaw_rate_rads - w_target_yaw_req_) * dt_s;
    
    // Simple PD for steering (Negative feedback to resist turning)
    // Adding a small deadband to yaw_rate to stop jittery tiny corrections
    float filtered_yaw_rate = (fabsf(yaw_rate_rads) < BALANCER_YAW_DEADBAND) ? 0.0f : yaw_rate_rads;
    float steer = -((yaw_error_accum_rad_ * configuration_.k_yaw) + (filtered_yaw_rate * configuration_.k_yaw_rate));

    // 3. Adaptive Trim (Navbot Pillar)
    if (configuration_.adaptive_trim_enabled) {
        // Only adapt if we have a significant position error to stop chasing noise
        if (fabsf(dist_err) > configuration_.adaptive_trim_deadband) {
            updateAdaptiveTrim(pitch_rad, dist_err, dt_s);
        }
    }

    // Convert to degrees for gain application (standard for this project)
    float theta_err_deg = (pitch_rad - pitch_trim_rad_) * (180.0f / M_PI);
    float pitch_rate_deg = lp_pitch_rate_ * (180.0f / M_PI);
    
    // 4. Control Law: u = Kp*theta + Kg*gyro + Kd*dist + Ks*speed
    // v71 FIX: Switching to '+' for Kd/Ks terms. If k_dist is positive and the robot drifts forward
    // but encoders count up, we need a negative command to go back. 
    // u = Kp*(th) + Kg*(gy) - Kd*(dist). 
    // In capture_15, '-Kd * dist' was showing as positive on the graph (red line), meaning it helped the fall.
    float term_angle = configuration_.k_pitch * theta_err_deg;
    float term_gyro  = configuration_.k_gyro  * pitch_rate_deg;
    float term_dist  = -configuration_.k_dist  * dist_err;
    float term_speed = -configuration_.k_speed * (v_enc_scaled - v_target_speed_);

    // Safety: Increase limits to allow the robot to fight back more strongly against drift.
    // Limits are now defined in lqr_config.h to avoid "magic numbers".
    term_dist = std::max(-BALANCER_LQR_SAT_DIST, std::min(BALANCER_LQR_SAT_DIST, term_dist));
    term_speed = std::max(-BALANCER_LQR_SAT_SPEED, std::min(BALANCER_LQR_SAT_SPEED, term_speed));

    float u = term_angle + term_gyro + term_dist + term_speed;

    // Optional command LPF
    if (configuration_.cmd_lpf_hz > 0.001f) {
        float tau_c = 1.0f / (2.0f * (float)M_PI * configuration_.cmd_lpf_hz);
        float alpha_c = dt_s / (tau_c + dt_s);
        lp_cmd_ = (lp_cmd_ * (1.0f - alpha_c)) + (u * alpha_c);
        u = lp_cmd_;
    } else {
        lp_cmd_ = u;
    }

    return {
        u, 
        steer,
        pitch_trim_rad_ * (180.0f / (float)M_PI),
        term_angle,
        term_gyro,
        term_dist,
        term_speed
    }; 
}

void IRAM_ATTR CascadedLqrStrategy::updateAdaptiveTrim(float pitch_rad, float distance_error, float delta_time_s) {
    // Navbot Approach: Only adapt when the robot is stable and passive.
    // Enhanced safety: Must be reasonably close to vertical and not under major correction.
    float pitch_deg = (pitch_rad - pitch_trim_rad_) * (180.0f / M_PI);
    if (fabsf(pitch_deg) > BALANCER_ADAPTIVE_TRIM_MAX_PITCH_DEG) {
        return;
    }

    // Must be stationary (no user command)
    if (fabsf(v_target_speed_) > 1.0f) {
        return;
    }

    // Must not be in a huge recovery (dist err small)
    if (fabsf(distance_error) > BALANCER_ADAPTIVE_TRIM_MAX_DIST_TICKS) {
        return;
    }

    // Integrated error logic: Adjust trim to zero-out the position drift over time.
    // v71 FIX: Sign was inverted. If dist_err is positive (forward), we need to lean BACK (negative trim)
    // to counteract the drift.
    pitch_trim_rad_ -= configuration_.adaptive_trim_alpha * distance_error * delta_time_s;
    
    // Clamp to configured limit
    const float limit = BALANCER_ADAPTIVE_TRIM_LIMIT_RAD;
    if (pitch_trim_rad_ > limit) {
        pitch_trim_rad_ = limit;
    }
    if (pitch_trim_rad_ < -limit) {
        pitch_trim_rad_ = -limit;
    }
}

void CascadedLqrStrategy::setDriveSetpoints(float v_norm, float w_norm) {
    v_target_speed_ = (v_norm * BALANCER_VELOCITY_TARGET_SCALE) / BALANCER_ENCODER_DOWNSCALE;
    w_target_yaw_req_ = w_norm;
}

void CascadedLqrStrategy::loadConfig() {
    Preferences prefs;
    if (prefs.begin("bal_lqr", true)) {
        loadConfigFromPreferences(prefs, configuration_);
        prefs.end();

        Config loadedConfiguration = configuration_;
        sanitizeConfigFilters(configuration_);
        if (loadedConfiguration.pitch_rate_lpf_hz != configuration_.pitch_rate_lpf_hz ||
            loadedConfiguration.cmd_lpf_hz != configuration_.cmd_lpf_hz) {
            LOG_PRINTF(abbot::log::CHANNEL_DEFAULT,
                       "LQR: sanitized unusable filter settings (pitch_rate_lpf_hz=%.3f->%.3f cmd_lpf_hz=%.3f->%.3f)\n",
                       (double)loadedConfiguration.pitch_rate_lpf_hz,
                       (double)configuration_.pitch_rate_lpf_hz,
                       (double)loadedConfiguration.cmd_lpf_hz,
                       (double)configuration_.cmd_lpf_hz);
        }

        LOG_PRINTF(abbot::log::CHANNEL_DEFAULT, "LQR: gains loaded (Kp=%.6f Kg=%.6f Kd=%.6f Ks=%.6f Ky=%.6f Kyr=%.6f)\n", 
                   (double)configuration_.k_pitch, (double)configuration_.k_gyro, (double)configuration_.k_dist, (double)configuration_.k_speed,
                   (double)configuration_.k_yaw, (double)configuration_.k_yaw_rate);
    }
}

void CascadedLqrStrategy::saveConfig() {
    Preferences prefs;
    if (prefs.begin("bal_lqr", false)) {
        saveConfigToPreferences(prefs, configuration_);
        prefs.end();
    }
}

void CascadedLqrStrategy::resetToDefaults() {
    configuration_.k_pitch = BALANCER_DEFAULT_K_PITCH;
    configuration_.k_gyro  = BALANCER_DEFAULT_K_GYRO;
    configuration_.k_dist  = BALANCER_DEFAULT_K_DIST;
    configuration_.k_speed = BALANCER_DEFAULT_K_SPEED;
    configuration_.k_yaw = BALANCER_DEFAULT_K_YAW;
    configuration_.k_yaw_rate = BALANCER_DEFAULT_K_YAW_RATE;
    configuration_.adaptive_trim_enabled = (BALANCER_ENABLE_ADAPTIVE_TRIM != 0);
    configuration_.adaptive_trim_alpha = BALANCER_ADAPTIVE_TRIM_ALPHA;
    configuration_.adaptive_trim_deadband = BALANCER_ADAPTIVE_TRIM_DEADBAND_TICKS;
    configuration_.pitch_rate_lpf_hz = BALANCER_LQR_PITCH_RATE_LPF_HZ;
    configuration_.cmd_lpf_hz = BALANCER_LQR_CMD_LPF_HZ;
    
    // Reset runtime state to prevent carrying over "learned" bias or filter states
    pitch_trim_rad_ = 0.0f;
    yaw_error_accum_rad_ = 0.0f;
    lp_pitch_rate_ = 0.0f;
    lp_v_speed_ = 0.0f;
    lp_cmd_ = 0.0f;
    needs_reset_enc_ = true;

    saveConfig();
}

void CascadedLqrStrategy::setConfig(const Config& configuration) {
    Config sanitizedConfiguration = configuration;
    Config originalConfiguration = sanitizedConfiguration;
    sanitizeConfigFilters(sanitizedConfiguration);
    if (configsEqual(configuration_, sanitizedConfiguration)) {
        return;
    }

    const bool trimChanged = (configuration_.adaptive_trim_enabled != sanitizedConfiguration.adaptive_trim_enabled);
    configuration_ = sanitizedConfiguration;
    if (!abbot::config::nearlyEqual(originalConfiguration.pitch_rate_lpf_hz, configuration_.pitch_rate_lpf_hz) ||
        !abbot::config::nearlyEqual(originalConfiguration.cmd_lpf_hz, configuration_.cmd_lpf_hz)) {
        LOG_PRINTF(abbot::log::CHANNEL_DEFAULT,
                   "LQR: rejected too-low filter settings (pitch_rate_lpf_hz=%.3f->%.3f cmd_lpf_hz=%.3f->%.3f)\n",
                   (double)originalConfiguration.pitch_rate_lpf_hz,
                   (double)configuration_.pitch_rate_lpf_hz,
                   (double)originalConfiguration.cmd_lpf_hz,
                   (double)configuration_.cmd_lpf_hz);
    }
    saveConfig();
    
    // If we just enabled trim, we might want to log it
    if (trimChanged) {
        LOG_PRINTF(abbot::log::CHANNEL_DEFAULT, "LQR: Adaptive Trim %s\n", configuration_.adaptive_trim_enabled ? "ENABLED" : "DISABLED");
    }
}

} // namespace balancing
} // namespace abbot
