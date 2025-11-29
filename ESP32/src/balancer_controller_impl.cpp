#include "balancer_controller.h"

// Implementation of the runtime controller that owns state, persistence and
// motor commanding. This was split out from the PID implementation to keep
// SystemTasks and the PID algorithm decoupled.

#include "logging.h"
#include "pid_controller.h"
#include "motor_driver.h"
#include "units.h"
#include <Preferences.h>

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

void init()
{
    g_pid.begin(BALANCER_DEFAULT_KP, BALANCER_DEFAULT_KI, BALANCER_DEFAULT_KD, BALANCER_INTEGRATOR_LIMIT);
    g_last_cmd = 0.0f;
    if (g_prefs.begin("abbot", false)) {
        g_prefs_started = true;
        float bp = g_prefs.getFloat("bp", BALANCER_DEFAULT_KP);
        float bi = g_prefs.getFloat("bi", BALANCER_DEFAULT_KI);
        float bd = g_prefs.getFloat("bd", BALANCER_DEFAULT_KD);
        g_pid.setGains(bp, bi, bd);
        g_deadband = g_prefs.getFloat("db", g_deadband);
        char buf[128];
        snprintf(buf, sizeof(buf), "BALANCER: controller initialized (Kp=%.4f Ki=%.4f Kd=%.4f deadband=%.4f)", (double)bp, (double)bi, (double)bd, (double)g_deadband);
        LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, buf);
    }
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
    char buf[128];
    snprintf(buf, sizeof(buf), "BALANCER: started (defaults used) - motors %s", abbot::motor::areMotorsEnabled() ? "ENABLED" : "NOT ENABLED");
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, buf);
}

void stop()
{
    if (!g_active) return;
    g_active = false;
    abbot::log::disableChannel(abbot::log::CHANNEL_BALANCER);
    abbot::motor::setMotorCommand(LEFT_MOTOR_ID, 0.0f);
    abbot::motor::setMotorCommand(RIGHT_MOTOR_ID, 0.0f);
    abbot::motor::disableMotors();
    g_pid.reset();
    g_last_cmd = 0.0f;
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
        g_prefs.putFloat("bp", kp);
        g_prefs.putFloat("bi", ki);
        g_prefs.putFloat("bd", kd);
    }
    char buf[128];
    snprintf(buf, sizeof(buf), "BALANCER: gains set Kp=%.4f Ki=%.4f Kd=%.4f (persisted=%s)", (double)kp, (double)ki, (double)kd, g_prefs_started ? "yes" : "no");
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, buf);
}

void getGains(float &kp, float &ki, float &kd)
{
    // Read persisted values if available, otherwise defaults
    if (g_prefs_started) {
        kp = g_prefs.getFloat("bp", BALANCER_DEFAULT_KP);
        ki = g_prefs.getFloat("bi", BALANCER_DEFAULT_KI);
        kd = g_prefs.getFloat("bd", BALANCER_DEFAULT_KD);
    } else {
        kp = BALANCER_DEFAULT_KP; ki = BALANCER_DEFAULT_KI; kd = BALANCER_DEFAULT_KD;
    }
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

// Called each IMU loop to compute and (if active) command motors. Returns
// the computed command (normalized) regardless of whether it was sent.
float processCycle(float fused_pitch, float fused_pitch_rate, float dt)
{
    if (!g_active) return 0.0f;
    // error: desired pitch 0 - measured
    float error = 0.0f - fused_pitch;
    float error_dot = -fused_pitch_rate;
    float cmd = g_pid.update(error, error_dot, dt);
    // clamp
    if (cmd > 1.0f) cmd = 1.0f;
    if (cmd < -1.0f) cmd = -1.0f;
    // slew
    float max_delta = g_cmd_slew * dt;
    float delta = cmd - g_last_cmd;
    if (delta > max_delta) delta = max_delta;
    if (delta < -max_delta) delta = -max_delta;
    cmd = g_last_cmd + delta;
    // deadband
    if (fabsf(cmd) > 0.0f && fabsf(cmd) < g_deadband) {
        cmd = (cmd > 0.0f) ? g_deadband : -g_deadband;
    }
    // if motors disabled, skip commanding but return computed value
    if (!abbot::motor::areMotorsEnabled()) {
        static bool warned = false;
        if (!warned) { LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "BALANCER: motors disabled - not commanding (use MOTOR ENABLE to arm)"); warned = true; }
        g_last_cmd = cmd;
        return cmd;
    }
    // command
    abbot::motor::setMotorCommand(LEFT_MOTOR_ID, cmd);
    abbot::motor::setMotorCommand(RIGHT_MOTOR_ID, cmd);
    LOG_PRINTF(abbot::log::CHANNEL_BALANCER, "BALANCER: cmd=%.4f err=%.6f pitch_deg=%.4f\n", cmd, (double)error, (double)radToDeg(fused_pitch));
    g_last_cmd = cmd;
    return cmd;
}

} // namespace controller
} // namespace balancer
} // namespace abbot

