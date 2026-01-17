#include "balancing/strategies/CascadedLqrStrategy.h"
#include "../../../config/balancer_config.h"
#include "logging.h"
#include <Preferences.h>
#include <algorithm>
#include <cmath>

namespace abbot {
namespace balancing {

CascadedLqrStrategy::CascadedLqrStrategy() {
    resetToDefaults();
}

void CascadedLqrStrategy::init() {
    loadConfig();
}

void CascadedLqrStrategy::reset(float initial_pitch_rad) {
    pitch_trim_rad_ = initial_pitch_rad;
    v_target_speed_ = 0.0f;
    w_target_yaw_req_ = 0.0f;
    yaw_error_accum_rad_ = 0.0f;
    needs_reset_enc_ = true;
}

IBalancingStrategy::Result CascadedLqrStrategy::compute(float pitch_rad, float pitch_rate_rads, float yaw_rate_rads, float dt_s,
                                 int32_t enc_l_ticks, int32_t enc_r_ticks,
                                 float v_enc_ticks_s) {
    
    // Low-pass filter the pitch rate to reduce high-frequency vibrations in motor commands
    // Alpha 0.2 at 1000Hz gives a cutoff of approx 35Hz (much smoother for mechanical stability).
    static float lp_pitch_rate = 0.0f;
    lp_pitch_rate = (lp_pitch_rate * 0.8f) + (pitch_rate_rads * 0.2f);

    // 1. Position tracking (LQR state x)
    int32_t avg_enc = (enc_l_ticks + enc_r_ticks) / 2;
    if (needs_reset_enc_) {
        enc_dist_zeropoint_ = avg_enc;
        needs_reset_enc_ = false;
        lp_pitch_rate = pitch_rate_rads;
    }
    float dist_err = (float)(avg_enc - enc_dist_zeropoint_);

    // 2. Yaw Tracking (Heading Hold)
    // Integrate yaw rate to get relative heading error from start
    yaw_error_accum_rad_ += (yaw_rate_rads - w_target_yaw_req_) * dt_s;
    
    // Simple PD for steering (Negative feedback to resist turning)
    // Adding a small deadband to yaw_rate to stop jittery tiny corrections
    float filtered_yaw_rate = (fabsf(yaw_rate_rads) < 0.01f) ? 0.0f : yaw_rate_rads;
    float steer = -((yaw_error_accum_rad_ * BALANCER_DEFAULT_K_YAW) + (filtered_yaw_rate * BALANCER_DEFAULT_K_YAW_RATE));

    // 3. Adaptive Trim (Navbot Pillar)
    if (cfg_.adaptive_trim_enabled) {
        updateAdaptiveTrim(pitch_rad, dist_err, dt_s);
    }

    // Convert to degrees for gain application (standard for this project)
    float theta_err_deg = (pitch_rad - pitch_trim_rad_) * (180.0f / M_PI);
    float pitch_rate_deg = lp_pitch_rate * (180.0f / M_PI);
    
    // 4. Control Law: u = Kp*theta + Kg*gyro - Kd*dist - Ks*speed
    float term_angle = cfg_.k_pitch * theta_err_deg;
    float term_gyro  = cfg_.k_gyro  * pitch_rate_deg;
    float term_dist  = -cfg_.k_dist  * dist_err;
    float term_speed = -cfg_.k_speed * (v_enc_ticks_s - v_target_speed_);

    // Safety: Limit the influence of position/speed terms to prevent them
    // from overpowering the pitch stability during a fall.
    term_dist = std::max(-0.2f, std::min(0.2f, term_dist));
    term_speed = std::max(-0.3f, std::min(0.3f, term_speed));

    float u = term_angle + term_gyro + term_dist + term_speed;

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

void CascadedLqrStrategy::updateAdaptiveTrim(float pitch_rad, float dist_err, float dt) {
    // Navbot Approach: Only adapt when the robot is stable and passive.
    // Enhanced safety: Must be very close to vertical and not under major correction.
    float pitch_deg = (pitch_rad - pitch_trim_rad_) * (180.0f / M_PI);
    if (fabsf(pitch_deg) > 1.0f) { // Stricter: 1 degree instead of 2
        return;
    }

    // Must be stationary (no user command)
    if (fabsf(v_target_speed_) > 1.0f) {
        return;
    }

    // Must not be in a huge recovery (dist err small)
    if (fabsf(dist_err) > 500.0f) {
        return;
    }

    // Integrated error logic: Adjust trim to zero-out the position drift over time.
    pitch_trim_rad_ -= cfg_.adaptive_trim_alpha * dist_err * dt;
    
    // Clamp to +/- 15 degrees (0.26 rad)
    const float limit = 0.26f;
    if (pitch_trim_rad_ > limit) {
        pitch_trim_rad_ = limit;
    }
    if (pitch_trim_rad_ < -limit) {
        pitch_trim_rad_ = -limit;
    }
}

void CascadedLqrStrategy::setDriveSetpoints(float v_norm, float w_norm) {
    v_target_speed_ = v_norm * 2000.0f; // Scale to ticks/s
    w_target_yaw_req_ = w_norm;
}

void CascadedLqrStrategy::loadConfig() {
    Preferences prefs;
    if (prefs.begin("bal_lqr", true)) {
        cfg_.k_pitch = prefs.getFloat("kp", BALANCER_DEFAULT_K_PITCH);
        cfg_.k_gyro  = prefs.getFloat("kg", BALANCER_DEFAULT_K_GYRO);
        cfg_.k_dist  = prefs.getFloat("kd", BALANCER_DEFAULT_K_DIST);
        cfg_.k_speed = prefs.getFloat("ks", BALANCER_DEFAULT_K_SPEED);
        cfg_.adaptive_trim_enabled = prefs.getBool("ate", (BALANCER_ENABLE_ADAPTIVE_TRIM != 0));
        cfg_.adaptive_trim_alpha = prefs.getFloat("ata", BALANCER_ADAPTIVE_TRIM_ALPHA);
        prefs.end();
        LOG_PRINTF(abbot::log::CHANNEL_DEFAULT, "LQR: gains loaded (Kp=%.6f Kg=%.6f Kd=%.6f Ks=%.6f)\n", 
                   (double)cfg_.k_pitch, (double)cfg_.k_gyro, (double)cfg_.k_dist, (double)cfg_.k_speed);
    }
}

void CascadedLqrStrategy::saveConfig() {
    Preferences prefs;
    if (prefs.begin("bal_lqr", false)) {
        prefs.putFloat("kp", cfg_.k_pitch);
        prefs.putFloat("kg", cfg_.k_gyro);
        prefs.putFloat("kd", cfg_.k_dist);
        prefs.putFloat("ks", cfg_.k_speed);
        prefs.putBool("ate", cfg_.adaptive_trim_enabled);
        prefs.putFloat("ata", cfg_.adaptive_trim_alpha);
        prefs.end();
    }
}

void CascadedLqrStrategy::resetToDefaults() {
    cfg_.k_pitch = BALANCER_DEFAULT_K_PITCH;
    cfg_.k_gyro  = BALANCER_DEFAULT_K_GYRO;
    cfg_.k_dist  = BALANCER_DEFAULT_K_DIST;
    cfg_.k_speed = BALANCER_DEFAULT_K_SPEED;
    cfg_.adaptive_trim_enabled = (BALANCER_ENABLE_ADAPTIVE_TRIM != 0);
    cfg_.adaptive_trim_alpha = BALANCER_ADAPTIVE_TRIM_ALPHA;
    saveConfig();
}

void CascadedLqrStrategy::setConfig(const Config& cfg) {
    bool trim_changed = (cfg_.adaptive_trim_enabled != cfg.adaptive_trim_enabled);
    cfg_ = cfg;
    saveConfig();
    
    // If we just enabled trim, we might want to log it
    if (trim_changed) {
        LOG_PRINTF(abbot::log::CHANNEL_DEFAULT, "LQR: Adaptive Trim %s\n", cfg_.adaptive_trim_enabled ? "ENABLED" : "DISABLED");
    }
}

} // namespace balancing
} // namespace abbot
