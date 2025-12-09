#include "balancer_controller.h"

// Implementation of the runtime controller that owns state, persistence and
// motor commanding. This was split out from the PID implementation to keep
// SystemTasks and the PID algorithm decoupled.

#include "logging.h"
#include "pid_controller.h"
#include "motor_driver.h"
#include "units.h"
#include "autotune_controller.h"
#include <Preferences.h>

#include "filter_manager.h"

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
    // Auto-enable motors only if upright
    float cur_pitch_deg = radToDeg(fused_pitch_rad);
    if (fabsf(cur_pitch_deg) <= BALANCER_AUTO_ENABLE_ANGLE_DEG) {
        abbot::motor::enableMotors();
    } else {
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
    snprintf(buf, sizeof(buf), "BALANCER: started (defaults used) - motors %s", abbot::motor::areMotorsEnabled() ? "ENABLED" : "NOT ENABLED");
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, buf);
}

void stop()
{
    if (!g_active) return;
    g_active = false;
    abbot::log::disableChannel(abbot::log::CHANNEL_BALANCER);
    abbot::motor::setMotorCommandBoth(0.0f, 0.0f);
    abbot::motor::disableMotors();
    g_pid.reset();
    g_last_cmd = 0.0f;
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
    if (now_ms - last_dbg_ms > 1000) {
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
            snprintf(dbg, sizeof(dbg), "AUTOTUNE: state=%d err=%.2f° cmd=%.3f motors_en=%d", 
                (int)g_autotune.getState(), (double)error_deg, (double)autotune_cmd, 
                abbot::motor::areMotorsEnabled() ? 1 : 0);
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
            abbot::motor::setMotorCommandBoth(0.0f, 0.0f);
            abbot::motor::disableMotors();
            LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "AUTOTUNE: motors disabled automatically");
            return 0.0f;
        }
        
        // Apply autotune command to motors
        if (abbot::motor::areMotorsEnabled()) {
            abbot::motor::setMotorCommandBoth(autotune_cmd, autotune_cmd);
        }
        
        return autotune_cmd;
    }
    
    if (!g_active) return 0.0f;
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
    float pid_out = computePidWithDriveSetpoint(fused_pitch, fused_pitch_rate, dt);
    float cmd = pid_out;  // No negation - same sign as pitch for catching behavior
    // clamp
    if (cmd > 1.0f) cmd = 1.0f;
    if (cmd < -1.0f) cmd = -1.0f;
    // slew (apply rate limit to prevent abrupt changes)
    float max_delta = g_cmd_slew * dt;
    float delta = cmd - g_last_cmd;
    if (delta > max_delta) delta = max_delta;
    if (delta < -max_delta) delta = -max_delta;
    cmd = g_last_cmd + delta;
    // deadband: only apply when command is non-zero after slew
    // If magnitude below threshold, zero it (allows smooth zero-crossing)
    // BUT: skip deadband if last_cmd was zero (startup/recovery) to allow initial response
    if (fabsf(g_last_cmd) > 0.001f && fabsf(cmd) < g_deadband) {
        cmd = 0.0f;
    }
    // if motors disabled, skip commanding but return computed value
    if (!abbot::motor::areMotorsEnabled()) {
        static bool warned = false;
        if (!warned) { LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "BALANCER: motors disabled - not commanding (use MOTOR ENABLE to arm)"); warned = true; }
        g_last_cmd = cmd;
        return cmd;
    }
    // command both motors simultaneously
    abbot::motor::setMotorCommandBoth(cmd, cmd);
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
    abbot::motor::enableMotors();
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
    abbot::motor::setMotorCommand(LEFT_MOTOR_ID, 0.0f);
    abbot::motor::setMotorCommand(RIGHT_MOTOR_ID, 0.0f);
    abbot::motor::disableMotors();
    
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

