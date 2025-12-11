#include "balancer_controller.h"

// Implementation of the runtime controller that owns state, persistence and
// motor commanding. This was split out from the PID implementation to keep
// SystemTasks and the PID algorithm decoupled.

#include "logging.h"
#include "pid_controller.h"
#include "motor_drivers/driver_manager.h"
#include "units.h"
#include "autotune_controller.h"
#include <Preferences.h>

#include "filter_manager.h"
#include "SystemTasks.h"

// Configs
#include "../config/balancer_config.h"
#include "../config/motor_config.h"

namespace abbot {
namespace balancer {
namespace controller {

static PIDController g_pid;
static bool g_active = false;
static float g_last_cmd = 0.0f;
static const float g_cmd_slew = BALANCER_CMD_SLEW_LIMIT;
static float g_deadband = BALANCER_MOTOR_MIN_OUTPUT;
static float g_pitch_trim_rad = 0.0f;
static Preferences g_prefs;
static bool g_prefs_started = false;
static AutotuneController g_autotune;
static bool g_autotune_active = false;
static AutotuneController::Config g_autocfg; // configurable autotune params
// Drive setpoints (high-level): normalized forward and turn commands
static float g_drive_target_v = 0.0f; // desired normalized forward [-1..1]
static float g_drive_target_w = 0.0f; // desired normalized turn [-1..1]
static float g_drive_v_filtered = 0.0f;
static float g_drive_last_pitch_sp = 0.0f;
// Configuration for drive->pitch mapping (from balancer_config.h)
static float g_drive_max_pitch_rad = degToRad(DRIVE_MAX_PITCH_DEG); // configured max pitch (deg)
static const float g_drive_v_slew = DRIVE_V_SLEW; // units per second (configured)
// Small non-blocking delay (ms) before enabling motors after a START
static const uint32_t g_enable_delay_ms = 250;
static uint32_t g_pending_enable_ts = 0;
static const float g_start_stable_angle_rad = degToRad(BALANCER_START_STABLE_ANGLE_DEG);
static const float g_start_stable_rate_rad_s = degToRad(BALANCER_START_STABLE_PITCH_RATE_DEG_S);
static const float g_fall_stop_angle_rad = degToRad(BALANCER_FALL_STOP_ANGLE_DEG);
static const float g_fall_stop_rate_rad_s = degToRad(BALANCER_FALL_STOP_RATE_DEG_S);
static const float g_trim_max_rad = degToRad(BALANCER_TRIM_MAX_DEG);

void init()
{
    g_pid.begin(BALANCER_DEFAULT_KP, BALANCER_DEFAULT_KI, BALANCER_DEFAULT_KD, BALANCER_INTEGRATOR_LIMIT);
    g_last_cmd = 0.0f;
    if (g_prefs.begin("abbot", false)) {
        g_prefs_started = true;
        char key[32];
        const char* fname = abbot::filter::getCurrentFilterName();
        // NVS keys are limited in length (typically 15 bytes). Build a safe
        // key by truncating the filter name if necessary so that suffixes like
        // "_bp" fit within limits.
        auto makePrefKey = [](char *out, size_t outsz, const char *fname, const char *suffix) {
            const size_t max_nvs_key = 15; // NVS key length limit
            size_t suf = strlen(suffix);
            size_t max_name = (suf >= max_nvs_key) ? 0 : (max_nvs_key - suf);
            // copy up to max_name characters from fname
            size_t copylen = strlen(fname);
            if (copylen > max_name) copylen = max_name;
            if (copylen > 0) {
                memcpy(out, fname, copylen);
                out[copylen] = '\0';
            } else {
                out[0] = '\0';
            }
            // append suffix
            strncat(out, suffix, outsz - strlen(out) - 1);
        };

        makePrefKey(key, sizeof(key), fname, "_bp");
        float bp = g_prefs.getFloat(key, BALANCER_DEFAULT_KP);
        makePrefKey(key, sizeof(key), fname, "_bi");
        float bi = g_prefs.getFloat(key, BALANCER_DEFAULT_KI);
        makePrefKey(key, sizeof(key), fname, "_bd");
        float bd = g_prefs.getFloat(key, BALANCER_DEFAULT_KD);
        g_pid.setGains(bp, bi, bd);
        // deadband remains global
        g_deadband = g_prefs.getFloat("db", g_deadband);
        char buf[128];
        snprintf(buf, sizeof(buf), "BALANCER: controller initialized (Kp=%.4f Ki=%.4f Kd=%.4f deadband=%.4f)", (double)bp, (double)bi, (double)bd, (double)g_deadband);
        LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, buf);
    } else {
        g_prefs_started = false;
        char buf[128];
        snprintf(buf, sizeof(buf), "BALANCER: controller initialized (defaults used) - Kp=%.4f Ki=%.4f Kd=%.4f deadband=%.4f", (double)BALANCER_DEFAULT_KP, (double)BALANCER_DEFAULT_KI, (double)BALANCER_DEFAULT_KD, (double)g_deadband);
        LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, buf);
    }
}

void setDriveSetpoints(float v_norm, float w_norm)
{
    // clamp inputs to [-1,1]
    if (v_norm > 1.0f) v_norm = 1.0f;
    if (v_norm < -1.0f) v_norm = -1.0f;
    if (w_norm > 1.0f) w_norm = 1.0f;
    if (w_norm < -1.0f) w_norm = -1.0f;
    g_drive_target_v = v_norm;
    g_drive_target_w = w_norm;
    // Log the requested setpoints for debugging
    LOG_PRINTF(abbot::log::CHANNEL_BALANCER, "SETDRIVE: v_req=%.3f w_req=%.3f\n", (double)g_drive_target_v, (double)g_drive_target_w);
}

void start(float fused_pitch_rad)
{
    if (g_active) return;
    g_active = true;
    abbot::log::enableChannel(abbot::log::CHANNEL_BALANCER);
    // Capture a small trim to compensate floor tilt/sensor bias. Clamp to avoid
    // masking large errors or upside-down starts.
    if (fabsf(fused_pitch_rad) > g_trim_max_rad) {
        g_pitch_trim_rad = copysignf(g_trim_max_rad, fused_pitch_rad);
    } else {
        g_pitch_trim_rad = fused_pitch_rad;
    }
    // Defer motor enabling briefly and only enable when fusion reports ready
    // and the robot is within the auto-enable angle. This avoids enabling
    // motors while the fusion/filter state is still settling after a stop
    // or other transient which can lead to immediate undesired motion.
    float cur_pitch_deg = radToDeg(fused_pitch_rad);
    if (fabsf(cur_pitch_deg) <= BALANCER_AUTO_ENABLE_ANGLE_DEG) {
        g_pending_enable_ts = millis() + g_enable_delay_ms;
        char msg[128];
        snprintf(msg, sizeof(msg), "BALANCER: start requested - delaying motor enable %lums (pitch=%.2fdeg)", (unsigned long)g_enable_delay_ms, (double)cur_pitch_deg);
        LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, msg);
    } else {
        g_pending_enable_ts = 0;
        char msg[128];
        snprintf(msg, sizeof(msg), "BALANCER: NOT enabling motors - pitch %.2f deg > limit %.1f deg", (double)cur_pitch_deg, (double)BALANCER_AUTO_ENABLE_ANGLE_DEG);
        LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, msg);
    }
    g_pid.reset();
    g_last_cmd = 0.0f;
    // Reset drive setpoint state on start to avoid carrying over joystick
    // commands or previous pitch setpoints which can cause the robot to
    // request a non-zero pitch immediately after re-enabling.
    g_drive_target_v = 0.0f;
    g_drive_target_w = 0.0f;
    g_drive_v_filtered = 0.0f;
    g_drive_last_pitch_sp = 0.0f;
    char buf[128];
    {
        bool men = false;
        if (auto d = abbot::motor::getActiveMotorDriver()) men = d->areMotorsEnabled();
        snprintf(buf, sizeof(buf), "BALANCER: started (defaults used) - motors %s", men ? "ENABLED" : "NOT ENABLED");
    }
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, buf);
}

void stop()
{
    if (!g_active) return;
    g_active = false;
    g_pitch_trim_rad = 0.0f;
    abbot::log::disableChannel(abbot::log::CHANNEL_BALANCER);
    if (auto d = abbot::motor::getActiveMotorDriver()) {
        d->setMotorCommandBoth(0.0f, 0.0f);
        d->disableMotors();
    }
    g_pid.reset();
    g_last_cmd = 0.0f;
    // Cancel any pending enable that may have been scheduled by start()
    g_pending_enable_ts = 0;
    // Clear drive setpoints when stopping so a subsequent start doesn't
    // immediately command a non-zero pitch based on previous joystick input.
    g_drive_target_v = 0.0f;
    g_drive_target_w = 0.0f;
    g_drive_v_filtered = 0.0f;
    g_drive_last_pitch_sp = 0.0f;
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "BALANCER: stopped - motors DISABLED");
}

bool isActive()
{
    return g_active;
}

void setGains(float kp, float ki, float kd)
{
    g_pid.setGains(kp, ki, kd);
    if (g_prefs_started) {
        char key[32];
        const char* fname = abbot::filter::getCurrentFilterName();
        auto makePrefKey = [](char *out, size_t outsz, const char *fname, const char *suffix) {
            const size_t max_nvs_key = 15; // NVS key length limit
            size_t suf = strlen(suffix);
            size_t max_name = (suf >= max_nvs_key) ? 0 : (max_nvs_key - suf);
            size_t copylen = strlen(fname);
            if (copylen > max_name) copylen = max_name;
            if (copylen > 0) {
                memcpy(out, fname, copylen);
                out[copylen] = '\0';
            } else {
                out[0] = '\0';
            }
            strncat(out, suffix, outsz - strlen(out) - 1);
        };
        makePrefKey(key, sizeof(key), fname, "_bp"); g_prefs.putFloat(key, kp);
        makePrefKey(key, sizeof(key), fname, "_bi"); g_prefs.putFloat(key, ki);
        makePrefKey(key, sizeof(key), fname, "_bd"); g_prefs.putFloat(key, kd);
    }
    char buf[128];
    snprintf(buf, sizeof(buf), "BALANCER: gains set Kp=%.4f Ki=%.4f Kd=%.4f (persisted=%s)", (double)kp, (double)ki, (double)kd, g_prefs_started ? "yes" : "no");
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, buf);
}

void getGains(float &kp, float &ki, float &kd)
{
    // Read persisted values if available, otherwise defaults
    if (g_prefs_started) {
        char key[32];
        const char* fname = abbot::filter::getCurrentFilterName();
        auto makePrefKey = [](char *out, size_t outsz, const char *fname, const char *suffix) {
            const size_t max_nvs_key = 15; // NVS key length limit
            size_t suf = strlen(suffix);
            size_t max_name = (suf >= max_nvs_key) ? 0 : (max_nvs_key - suf);
            size_t copylen = strlen(fname);
            if (copylen > max_name) copylen = max_name;
            if (copylen > 0) {
                memcpy(out, fname, copylen);
                out[copylen] = '\0';
            } else {
                out[0] = '\0';
            }
            strncat(out, suffix, outsz - strlen(out) - 1);
        };
        makePrefKey(key, sizeof(key), fname, "_bp"); kp = g_prefs.getFloat(key, BALANCER_DEFAULT_KP);
        makePrefKey(key, sizeof(key), fname, "_bi"); ki = g_prefs.getFloat(key, BALANCER_DEFAULT_KI);
        makePrefKey(key, sizeof(key), fname, "_bd"); kd = g_prefs.getFloat(key, BALANCER_DEFAULT_KD);
    } else {
        kp = BALANCER_DEFAULT_KP; ki = BALANCER_DEFAULT_KI; kd = BALANCER_DEFAULT_KD;
    }
}

void resetGainsToDefaults()
{
    g_pid.setGains(BALANCER_DEFAULT_KP, BALANCER_DEFAULT_KI, BALANCER_DEFAULT_KD);
    if (g_prefs_started) {
        char key[32];
        const char* fname = abbot::filter::getCurrentFilterName();
        auto makePrefKey = [](char *out, size_t outsz, const char *fname, const char *suffix) {
            const size_t max_nvs_key = 15; // NVS key length limit
            size_t suf = strlen(suffix);
            size_t max_name = (suf >= max_nvs_key) ? 0 : (max_nvs_key - suf);
            size_t copylen = strlen(fname);
            if (copylen > max_name) copylen = max_name;
            if (copylen > 0) {
                memcpy(out, fname, copylen);
                out[copylen] = '\0';
            } else {
                out[0] = '\0';
            }
            strncat(out, suffix, outsz - strlen(out) - 1);
        };
        makePrefKey(key, sizeof(key), fname, "_bp"); g_prefs.putFloat(key, BALANCER_DEFAULT_KP);
        makePrefKey(key, sizeof(key), fname, "_bi"); g_prefs.putFloat(key, BALANCER_DEFAULT_KI);
        makePrefKey(key, sizeof(key), fname, "_bd"); g_prefs.putFloat(key, BALANCER_DEFAULT_KD);
        // preserve legacy global deadband key
        g_prefs.putFloat("db", BALANCER_MOTOR_MIN_OUTPUT);
    }
    g_deadband = BALANCER_MOTOR_MIN_OUTPUT;
    char buf[128];
    snprintf(buf, sizeof(buf), "BALANCER: reset to defaults Kp=%.4f Ki=%.4f Kd=%.4f deadband=%.4f", 
             (double)BALANCER_DEFAULT_KP, (double)BALANCER_DEFAULT_KI, (double)BALANCER_DEFAULT_KD, (double)BALANCER_MOTOR_MIN_OUTPUT);
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, buf);
}

void setDeadband(float db)
{
    g_deadband = db;
    if (g_prefs_started) g_prefs.putFloat("db", g_deadband);
    char buf[128]; snprintf(buf, sizeof(buf), "BALANCER: deadband set to %.6f (persisted=%s)", (double)g_deadband, g_prefs_started ? "yes" : "no");
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, buf);
}

float getDeadband()
{
    return g_deadband;
}

void calibrateDeadband()
{
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "DEADBAND calibration not yet implemented. Use 'BALANCE DEADBAND SET <v>' to store a value.");
}

// Helper: apply drive->pitch mapping with slew limiting and compute PID output
// Returns the raw PID controller output (not yet slew/ deadband/clamped for motor)
static float computePidWithDriveSetpoint(float fused_pitch, float fused_pitch_rate, float dt)
{
    // Filter normalized v toward target with simple first-order slew limiter
    if (g_drive_v_filtered != g_drive_target_v) {
        float max_dv = g_drive_v_slew * dt;
        float dv = g_drive_target_v - g_drive_v_filtered;
        if (dv > max_dv) dv = max_dv;
        if (dv < -max_dv) dv = -max_dv;
        g_drive_v_filtered += dv;
    }
    // pitch setpoint from filtered velocity command
    float pitch_sp = g_drive_v_filtered * g_drive_max_pitch_rad;
    // clamp pitch setpoint
    if (pitch_sp > g_drive_max_pitch_rad) pitch_sp = g_drive_max_pitch_rad;
    if (pitch_sp < -g_drive_max_pitch_rad) pitch_sp = -g_drive_max_pitch_rad;
    // estimate pitch_sp rate
    float pitch_sp_rate = 0.0f;
    if (dt > 0.0f) {
        pitch_sp_rate = (pitch_sp - g_drive_last_pitch_sp) / dt;
    }
    g_drive_last_pitch_sp = pitch_sp;

    // PID input: error = fused_pitch - pitch_sp (target = pitch_sp)
    float pid_in = fused_pitch - pitch_sp;
    float pid_rate = fused_pitch_rate - pitch_sp_rate;
    float pid_out = g_pid.update(pid_in, pid_rate, dt);

    // Rate-limited debug logging to avoid spamming serial
    static uint32_t last_dbg_ms = 0;
    uint32_t now_ms = millis();
    if (now_ms - last_dbg_ms > 20) {
        LOG_PRINTF(abbot::log::CHANNEL_BALANCER,
                   "DRIVE DBG tgtV=%.3f filtV=%.3f pitch_sp=%.3fdeg pitch_sp_rate=%.3fdeg/s pid_in=%.3fdeg pid_rate=%.3fdeg/s pid_out=%.3f\n",
                   (double)g_drive_target_v,
                   (double)g_drive_v_filtered,
                   (double)radToDeg(pitch_sp),
                   (double)radToDeg(pitch_sp_rate),
                   (double)radToDeg(pid_in),
                   (double)radToDeg(pid_rate),
                   (double)pid_out);
        last_dbg_ms = now_ms;
    }

    return pid_out;
}

// Called each IMU loop to compute and (if active) command motors. Returns
// the computed command (normalized) regardless of whether it was sent.
// Note: Pitch sign is now handled by axis mapping in FusionConfig (accel_sign/gyro_sign),
// so pitch arrives with correct sign: positive = tilted forward.
float processCycle(float fused_pitch, float fused_pitch_rate, float dt)
{
    // Check if autotuning is active
    if (g_autotune_active && g_autotune.isActive()) {
        // Pitch is already correctly signed from fusion
        float error_rad = fused_pitch;
        float error_deg = radToDeg(error_rad);
        
        uint32_t dt_ms = (uint32_t)(dt * 1000.0f);
        
        // Autotune relay output - use directly (same sign convention as PID)
        float autotune_raw = g_autotune.update(error_deg, dt_ms);
        float autotune_cmd = autotune_raw;  // No inversion - consistent with PID
        
        // Debug: log autotune state and command
        static uint32_t last_log_ms = 0;
        uint32_t now_ms = millis();
        if (now_ms - last_log_ms > 500) {
            char dbg[128];
            int men = 0; if (auto d = abbot::motor::getActiveMotorDriver()) men = d->areMotorsEnabled() ? 1 : 0;
            snprintf(dbg, sizeof(dbg), "AUTOTUNE: state=%d err=%.2f° cmd=%.3f motors_en=%d", 
                (int)g_autotune.getState(), (double)error_deg, (double)autotune_cmd, men);
            LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, dbg);
            last_log_ms = now_ms;
        }
        
        // Check if autotune just finished
        if (!g_autotune.isActive()) {
            g_autotune_active = false;
            
            if (g_autotune.getState() == AutotuneController::State::COMPLETE) {
                LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "AUTOTUNE: COMPLETE - use 'AUTOTUNE APPLY' to set gains");
            } else if (g_autotune.getState() == AutotuneController::State::FAILED) {
                char buf[128];
                snprintf(buf, sizeof(buf), "AUTOTUNE: FAILED - %s", g_autotune.getFailureReason());
                LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, buf);
            }
            
            // Zero and disable motors
            if (auto d = abbot::motor::getActiveMotorDriver()) { d->setMotorCommandBoth(0.0f, 0.0f); d->disableMotors(); }
            LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "AUTOTUNE: motors disabled automatically");
            return 0.0f;
        }
        
        // Apply autotune command to motors
        if (auto d = abbot::motor::getActiveMotorDriver()) {
            if (d->areMotorsEnabled()) d->setMotorCommandBoth(autotune_cmd, autotune_cmd);
        }
        
        return autotune_cmd;
    }
    
    if (!g_active) return 0.0f;

    // Fall-detection guard: if the robot is clearly down, stop the balancer to
    // avoid fighting on the ground. Uses both angle and optional rate threshold.
    if (fabsf(fused_pitch) > g_fall_stop_angle_rad ||
        (g_fall_stop_rate_rad_s > 0.0f && fabsf(fused_pitch_rate) > g_fall_stop_rate_rad_s)) {
        stop();
        LOG_PRINTF(abbot::log::CHANNEL_DEFAULT,
                   "BALANCER: auto-stopped (fell) pitch=%.2fdeg rate=%.2fdeg/s",
                   (double)radToDeg(fused_pitch), (double)radToDeg(fused_pitch_rate));
        return 0.0f;
    }
    // If a motor enable was deferred by start(), check whether it's time to
    // attempt enabling. Only enable when fusion reports ready; otherwise
    // cancel the pending enable to avoid enabling into a transient.
    if (g_pending_enable_ts != 0) {
        uint32_t now = millis();
        if ((int32_t)(now - g_pending_enable_ts) >= 0) {
            float cur_pitch_deg = radToDeg(fused_pitch);
            if (!abbot::isFusionReady()) {
                LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "BALANCER: delayed enable deferred - fusion not ready");
                g_pending_enable_ts = now + g_enable_delay_ms;
            } else if (fabsf(fused_pitch) > g_start_stable_angle_rad) {
                char msg[128];
                snprintf(msg, sizeof(msg), "BALANCER: delayed enable deferred - pitch %.2f deg outside %.2f deg", (double)cur_pitch_deg, (double)radToDeg(g_start_stable_angle_rad));
                LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, msg);
                g_pending_enable_ts = now + g_enable_delay_ms;
            } else if (fabsf(fused_pitch_rate) > g_start_stable_rate_rad_s) {
                char msg[128];
                snprintf(msg, sizeof(msg), "BALANCER: delayed enable deferred - pitch rate %.2f deg/s > %.2f deg/s", (double)radToDeg(fused_pitch_rate), (double)radToDeg(g_start_stable_rate_rad_s));
                LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, msg);
                g_pending_enable_ts = now + g_enable_delay_ms;
            } else {
                    if (auto d = abbot::motor::getActiveMotorDriver()) d->enableMotors();
                    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "BALANCER: motors ENABLED after settle");
                g_pending_enable_ts = 0;
            }
        }
    }
    // For INVERTED PENDULUM (balancing robot):
    // - Pitch > 0 (tilted forward) -> motors must go FORWARD (cmd > 0) to catch
    // - Pitch < 0 (tilted backward) -> motors must go BACKWARD (cmd < 0) to catch
    // With ay > 0 when tilted forward and our axis mapping, Madgwick produces
    // pitch that may be positive or negative for forward tilt depending on convention.
    // We use positive command = motors forward.
    //
    // NOTE: Motor directions are CORRECT (validated with MOTOR VEL commands).
    //       DO NOT change motor inversion in motor_config.h!
    //       Pitch sign is handled by accel_sign/gyro_sign in FusionConfig.h
    
    // Compute PID output taking into account the drive-setpoint->pitch mapping
    float pitch_for_control = fused_pitch - g_pitch_trim_rad;
    float pid_out = computePidWithDriveSetpoint(pitch_for_control, fused_pitch_rate, dt);
    // Compute pid_in here (pitch error) for logging (pitch_sp computed from filtered drive)
    float pitch_sp = g_drive_v_filtered * g_drive_max_pitch_rad;
    float pid_in = pitch_for_control - pitch_sp;
    float cmd = pid_out;  // No negation - same sign as pitch for catching behavior
    // clamp (pre-slew). Keep a copy for logging to inspect clamp vs slew behavior
    float cmd_pre_slew = cmd;
    if (cmd_pre_slew > 1.0f) cmd_pre_slew = 1.0f;
    if (cmd_pre_slew < -1.0f) cmd_pre_slew = -1.0f;
    // apply the clamp to cmd (we'll apply slew after)
    if (cmd > 1.0f) cmd = 1.0f;
    if (cmd < -1.0f) cmd = -1.0f;
    // slew (apply rate limit to prevent abrupt changes)
    float max_delta = g_cmd_slew * dt;
    float delta = cmd - g_last_cmd;
    if (delta > max_delta) delta = max_delta;
    if (delta < -max_delta) delta = -max_delta;
    cmd = g_last_cmd + delta;
    // Rate-limited debug logging for PID vs final commanded value (every ~200ms)
    static uint32_t last_cmd_dbg_ms = 0;
    uint32_t now_cmd_dbg_ms = millis();
    if ((int32_t)(now_cmd_dbg_ms - last_cmd_dbg_ms) >= 200) {
        LOG_PRINTF(abbot::log::CHANNEL_BALANCER,
                   "BALANCER_DBG pitch=%.2fdeg pitch_rate=%.2fdeg/s pid_in=%.3f pid_out=%.3f cmd_pre_slew=%.3f cmd_after_slew=%.3f last_cmd=%.3f\n",
                   (double)radToDeg(fused_pitch), (double)radToDeg(fused_pitch_rate), (double)radToDeg(pid_in), (double)pid_out,
                   (double)cmd_pre_slew, (double)cmd, (double)g_last_cmd);
        last_cmd_dbg_ms = now_cmd_dbg_ms;
    }
    // deadband: only apply when command is non-zero after slew
    // If magnitude below threshold, zero it (allows smooth zero-crossing)
    // BUT: skip deadband if last_cmd was zero (startup/recovery) to allow initial response
    if (fabsf(g_last_cmd) > 0.001f && fabsf(cmd) < g_deadband) {
        cmd = 0.0f;
    }
    // if motors disabled, skip commanding but return computed value
    if (auto d = abbot::motor::getActiveMotorDriver()) {
        if (!d->areMotorsEnabled()) {
            static bool warned = false;
            if (!warned) { LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "BALANCER: motors disabled - not commanding (use MOTOR ENABLE to arm)"); warned = true; }
            g_last_cmd = cmd;
            return cmd;
        }
        // command both motors simultaneously
        d->setMotorCommandBoth(cmd, cmd);
    } else {
        static bool warned_no_drv = false;
        if (!warned_no_drv) { LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "BALANCER: no active motor driver - not commanding"); warned_no_drv = true; }
        g_last_cmd = cmd;
        return cmd;
    }
    // LOG_PRINTF(abbot::log::CHANNEL_BALANCER, "BALANCER: cmd=%.4f err=%.6f pitch_deg=%.4f\n", cmd, (double)error, (double)radToDeg(fused_pitch));
    g_last_cmd = cmd;
    return cmd;
}

void startAutotune()
{
    if (g_autotune_active) {
        LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "AUTOTUNE: already running");
        return;
    }
    
    // Stop regular balancing
    if (g_active) {
        LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "AUTOTUNE: stopping balancer to start tuning");
        stop();
    }
    
    // Start with current configurable parameters (defaults in g_autocfg)
    g_autotune.start(&g_autocfg);
    g_autotune_active = true;
    
    // Always enable motors for autotuning
    if (auto d = abbot::motor::getActiveMotorDriver()) d->enableMotors();
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "AUTOTUNE: motors enabled automatically");
    
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "AUTOTUNE: started - applying relay control (HOLD THE ROBOT!)");
}

void stopAutotune()
{
    if (!g_autotune_active) {
        LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "AUTOTUNE: not running");
        return;
    }
    
    g_autotune.stop();
    g_autotune_active = false;
    
    // Zero and disable motors
    if (auto d = abbot::motor::getActiveMotorDriver()) {
        d->setMotorCommand(LEFT_MOTOR_ID, 0.0f);
        d->setMotorCommand(RIGHT_MOTOR_ID, 0.0f);
        d->disableMotors();
    }
    
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "AUTOTUNE: stopped - motors disabled");
}

bool isAutotuning()
{
    return g_autotune_active && g_autotune.isActive();
}

const char* getAutotuneStatus()
{
    if (!g_autotune_active) {
        return "IDLE";
    }
    
    switch (g_autotune.getState()) {
        case AutotuneController::State::IDLE:
            return "IDLE";
        case AutotuneController::State::WAITING_START:
            return "WAITING_START (apply disturbance)";
        case AutotuneController::State::COLLECTING:
            return "COLLECTING (measuring oscillations)";
        case AutotuneController::State::ANALYZING:
            return "ANALYZING (computing gains)";
        case AutotuneController::State::COMPLETE:
            return "COMPLETE (use AUTOTUNE APPLY to set gains)";
        case AutotuneController::State::FAILED:
            return g_autotune.getFailureReason();
        default:
            return "UNKNOWN";
    }
}

void applyAutotuneGains()
{
    if (g_autotune.getState() != AutotuneController::State::COMPLETE) {
        LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "AUTOTUNE: no successful tuning result available");
        return;
    }
    
    const auto& result = g_autotune.getResult();
    if (!result.success) {
        LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "AUTOTUNE: tuning failed");
        return;
    }
    
    // Apply the computed gains
    // IMPORTANT: Autotune calculates gains based on DEGREES (Input=Deg, Output=0..1)
    // But the PID controller works in RADIANS (Input=Rad, Output=0..1)
    // We must convert the gains: K_rad = K_deg * (180/PI)
    const float deg2rad_factor = 180.0f / M_PI;
    float Kp = result.Kp * deg2rad_factor;
    float Ki = result.Ki * deg2rad_factor;
    float Kd = result.Kd * deg2rad_factor;

    setGains(Kp, Ki, Kd);
    
    char buf[256];
    snprintf(buf, sizeof(buf), 
        "AUTOTUNE: Applied gains (Rad) - Kp=%.4f Ki=%.4f Kd=%.4f | (Raw Deg: Kp=%.5f)",
        (double)Kp, (double)Ki, (double)Kd, (double)result.Kp);
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, buf);
    
    g_autotune_active = false;
}

// --- Autotune configuration setters ---
void setAutotuneRelay(float amplitude)
{
    if (amplitude < 0.05f) amplitude = 0.05f;
    if (amplitude > 1.0f) amplitude = 1.0f;
    g_autocfg.relay_amplitude = amplitude;
    char buf[128]; snprintf(buf, sizeof(buf), "AUTOTUNE: relay amplitude set to %.3f", (double)amplitude);
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, buf);
}

void setAutotuneDeadband(float deadband_deg)
{
    if (deadband_deg < 0.0f) deadband_deg = 0.0f;
    if (deadband_deg > 10.0f) deadband_deg = 10.0f;
    g_autocfg.deadband = deadband_deg;
    char buf[128]; snprintf(buf, sizeof(buf), "AUTOTUNE: deadband set to %.3f°", (double)deadband_deg);
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, buf);
}

void setAutotuneMaxAngle(float max_pitch_deg)
{
    if (max_pitch_deg < 5.0f) max_pitch_deg = 5.0f;
    if (max_pitch_deg > 90.0f) max_pitch_deg = 90.0f;
    g_autocfg.max_pitch_abort = max_pitch_deg;
    char buf[128]; snprintf(buf, sizeof(buf), "AUTOTUNE: max angle set to %.1f°", (double)max_pitch_deg);
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, buf);
}

} // namespace controller
} // namespace balancer
} // namespace abbot

