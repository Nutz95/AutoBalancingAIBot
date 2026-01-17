#include "balancer_controller.h"

// Implementation of the runtime controller that owns state, persistence and
// motor commanding.
// This module now delegates balancing logic to abbot::balancing::BalancingManager
// which supports multiple strategies (Legacy PID, Cascaded LQR).

#include "balancing/BalancingManager.h"
#include "balancing/strategies/LegacyPidStrategy.h"
#include "balancing/strategies/CascadedLqrStrategy.h"

#include "autotune_controller.h"
#include "logging.h"
#include "imu_drivers/imu_manager.h"
#include "motor_drivers/driver_manager.h"
#include "units.h"
#include <Preferences.h>

#include "SystemTasks.h"
#include "filter_manager.h"

// Configs
#include "../config/balancer_config.h"
#include "../config/motor_configs/servo_motor_config.h"
#include <cstdio>
#include <algorithm>
#include <cmath>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

namespace abbot {
namespace balancer {
namespace controller {

static bool g_active = false;
static float g_last_cmd = 0.0f;
static const float g_command_slew_rate = BALANCER_CMD_SLEW_LIMIT;
static float g_deadband = BALANCER_MOTOR_MIN_OUTPUT;
static float g_min_command_output = BALANCER_MIN_CMD; 
static Preferences g_preferences_helper;
static bool g_prefs_ready = false;

static AutotuneController g_autotune_engine;
static bool g_autotune_active = false;
static AutotuneController::Config g_autotune_cfg; 

// Motor gain scaling (for asymmetric compensation)
static float g_left_motor_scale = BALANCER_LEFT_MOTOR_GAIN;
static float g_right_motor_scale = BALANCER_RIGHT_MOTOR_GAIN;

// Adaptive start thresholds
static bool g_adaptive_start_active = false;
static float g_start_throttle_left = 0.0f;
static float g_start_throttle_right = 0.0f;

// Latest IMU sample (robot frame)
static float g_latest_accel[3] = {0.0f, 0.0f, 0.0f};
static float g_latest_gyro[3] = {0.0f, 0.0f, 0.0f}; // rad/s

struct TelemetryState {
  int32_t last_enc_l = 0;
  int32_t last_enc_r = 0;
  uint32_t last_update_us = 0;
  float vel_l_ticks_s = 0.0f;
  float vel_r_ticks_s = 0.0f;
  bool is_init = false;
};

static TelemetryState g_telemetry_state;
static SemaphoreHandle_t g_telemetry_mutex = nullptr;
static TaskHandle_t g_telemetry_task = nullptr;

static void pollEncoders(float /*dt_sec*/) {
    auto driver = abbot::motor::getActiveMotorDriver();
    if (!driver) return;
    
    uint32_t now_us = (uint32_t)esp_timer_get_time();
    int32_t l = 0, r = 0;
    driver->readEncodersBoth(l, r);
    
    if (g_telemetry_mutex && xSemaphoreTake(g_telemetry_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        if (g_telemetry_state.is_init) {
            float dt = (float)(now_us - g_telemetry_state.last_update_us) / 1000000.0f;
            if (dt > 0.0001f) {
                g_telemetry_state.vel_l_ticks_s = (float)(l - g_telemetry_state.last_enc_l) / dt;
                g_telemetry_state.vel_r_ticks_s = (float)(r - g_telemetry_state.last_enc_r) / dt;
            }
        }
        g_telemetry_state.last_enc_l = l;
        g_telemetry_state.last_enc_r = r;
        g_telemetry_state.last_update_us = now_us;
        g_telemetry_state.is_init = true;
        xSemaphoreGive(g_telemetry_mutex);
    }
}

static void telemetryTask(void *pv) {
    while (true) {
        pollEncoders(0.0f);
        vTaskDelay(pdMS_TO_TICKS(BALANCER_ENCODER_UPDATE_MS > 0 ? BALANCER_ENCODER_UPDATE_MS : 20));
    }
}

static float g_target_forward = 0.0f;
static float g_target_turn = 0.0f;
static uint32_t g_auto_enable_ts = 0;
static int g_prev_cmd_sign = 0;

void init() {
  if (!g_telemetry_mutex) g_telemetry_mutex = xSemaphoreCreateMutex();
  if (!g_telemetry_task) {
    xTaskCreatePinnedToCore(telemetryTask, "bal_tel", 4096, nullptr, 1, &g_telemetry_task, 0);
  }

  auto& manager = abbot::balancing::BalancingManager::getInstance();
  manager.init();

  if (g_preferences_helper.begin("abbot", false)) {
    g_prefs_ready = true;
    g_deadband = g_preferences_helper.getFloat("db", BALANCER_MOTOR_MIN_OUTPUT);
    g_min_command_output = g_preferences_helper.getFloat("min_cmd", BALANCER_MIN_CMD);
    
    // Load and apply saved strategy
    int saved_mode = g_preferences_helper.getInt("mode", (int)ControllerMode::LEGACY_PID);
    manager.setStrategy((abbot::balancing::StrategyType)saved_mode);

    g_adaptive_start_active = g_preferences_helper.getBool("adaptive_st", false);
    g_start_throttle_left = g_preferences_helper.getFloat("st_L", 0.0f);
    g_start_throttle_right = g_preferences_helper.getFloat("st_R", 0.0f);
    g_left_motor_scale = g_preferences_helper.getFloat("mg_L", BALANCER_LEFT_MOTOR_GAIN);
    g_right_motor_scale = g_preferences_helper.getFloat("mg_R", BALANCER_RIGHT_MOTOR_GAIN);

    LOG_PRINTF(abbot::log::CHANNEL_DEFAULT, "BALANCER: init (Mode=%s, DB=%.3f, Trim=%s)\n", 
               manager.getActiveStrategy()->getName(), (double)g_deadband,
               isAdaptiveTrimEnabled() ? "ON" : "OFF");
  }
}

void setDriveSetpoints(float v, float w) {
  g_target_forward = std::max(-1.0f, std::min(1.0f, v));
  g_target_turn = std::max(-1.0f, std::min(1.0f, w));
  abbot::balancing::BalancingManager::getInstance().setDriveSetpoints(g_target_forward, g_target_turn);
}

void start(float pitch_rad) {
  if (g_active) return;
  g_active = true;
  abbot::log::enableChannel(abbot::log::CHANNEL_BALANCER);
  
  auto& manager = abbot::balancing::BalancingManager::getInstance();
  manager.reset(pitch_rad);
  
  if (fabsf(radToDeg(pitch_rad)) <= BALANCER_AUTO_ENABLE_ANGLE_DEG) {
    g_auto_enable_ts = millis() + 250;
  } else {
    g_auto_enable_ts = 0;
  }

  g_last_cmd = 0.0f;
  abbot::reinitFilterFromAccel();
  LOG_PRINTF(abbot::log::CHANNEL_DEFAULT, "BALANCER: started (%s)\n", manager.getActiveStrategy()->getName());
}

void stop() {
  if (!g_active) return;
  g_active = false;
  abbot::log::disableChannel(abbot::log::CHANNEL_BALANCER);
  if (auto driver = abbot::motor::getActiveMotorDriver()) {
    driver->setMotorCommandBoth(0.0f, 0.0f);
    driver->disableMotors();
  }
  g_last_cmd = 0.0f;
  g_auto_enable_ts = 0;
  LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "BALANCER: stopped");
}

bool isActive() { return g_active; }

void setGains(float kp, float ki, float kd) {
  auto* s = abbot::balancing::BalancingManager::getInstance().getStrategy(abbot::balancing::StrategyType::LEGACY_PID);
  if (s) {
    static_cast<abbot::balancing::LegacyPidStrategy*>(s)->setGains(kp, ki, kd);
  }
}

void getGains(float &kp, float &ki, float &kd) {
  auto* s = abbot::balancing::BalancingManager::getInstance().getStrategy(abbot::balancing::StrategyType::LEGACY_PID);
  if (s) {
    static_cast<abbot::balancing::LegacyPidStrategy*>(s)->getGains(kp, ki, kd);
  } else {
    kp = BALANCER_DEFAULT_KP; 
    ki = BALANCER_DEFAULT_KI; 
    kd = BALANCER_DEFAULT_KD;
  }
}

void resetGainsToDefaults() {
  abbot::balancing::BalancingManager::getInstance().getActiveStrategy()->resetToDefaults();
}

void setDeadband(float db) {
  g_deadband = db;
  if (g_prefs_ready) g_preferences_helper.putFloat("db", db);
}

float getDeadband() { return g_deadband; }

void setMinCmd(float m) {
  g_min_command_output = m;
  if (g_prefs_ready) g_preferences_helper.putFloat("min_cmd", m);
}

float getMinCmd() { return g_min_command_output; }

void setLatestImuSample(const float a[3], const float g[3]) {
  if (a) memcpy(g_latest_accel, a, sizeof(g_latest_accel));
  if (g) memcpy(g_latest_gyro, g, sizeof(g_latest_gyro));
}

float processCycle(float pitch, float pitch_rate, float dt) {
  if (g_autotune_active) {
      // Minimal autotune integration for compatibility (simplified)
      float out = g_autotune_engine.update(radToDeg(pitch), dt);
      if (!g_autotune_engine.isActive()) g_autotune_active = false;
      if (auto d = abbot::motor::getActiveMotorDriver()) {
          d->setMotorCommandBoth(out * g_left_motor_scale, out * g_right_motor_scale);
      }
      return out;
  }

  if (!g_active) return 0.0f;

  // Fall check
  if (fabsf(pitch) > degToRad(BALANCER_FALL_STOP_ANGLE_DEG)) {
    stop();
    return 0.0f;
  }

  // Auto-enable
  if (g_auto_enable_ts > 0 && (int32_t)(millis() - g_auto_enable_ts) >= 0) {
    if (auto d = abbot::motor::getActiveMotorDriver()) d->enableMotors();
    g_auto_enable_ts = 0;
  }

  // Strategy compute
  int32_t el = 0, er = 0;
  float v_enc = 0.0f;
  if (g_telemetry_mutex && xSemaphoreTake(g_telemetry_mutex, 0) == pdTRUE) {
    el = g_telemetry_state.last_enc_l;
    er = g_telemetry_state.last_enc_r;
    v_enc = (g_telemetry_state.vel_l_ticks_s + g_telemetry_state.vel_r_ticks_s) / 2.0f;
    xSemaphoreGive(g_telemetry_mutex);
  }

  float cmd = abbot::balancing::BalancingManager::getInstance().compute(pitch, pitch_rate, dt, el, er, v_enc);

  // Slew
  float max_d = g_command_slew_rate * dt;
  cmd = std::max(g_last_cmd - max_d, std::min(g_last_cmd + max_d, cmd));
  cmd = std::max(-1.0f, std::min(1.0f, cmd));

  // Deadband
  if (fabsf(cmd) < g_deadband) {
    if (fabsf(cmd) < 0.001f) cmd = 0.0f;
    else cmd = (cmd > 0 ? 1.0f : -1.0f) * g_min_command_output;
  }

  g_last_cmd = cmd;

  if (auto d = abbot::motor::getActiveMotorDriver()) {
    if (d->areMotorsEnabled()) {
      d->setMotorCommandBoth(cmd * g_left_motor_scale, cmd * g_right_motor_scale);
    }
  }

  return cmd;
}

// Stubs for remaining required exports
void startAutotune() { 
    g_autotune_engine.start(&g_autotune_cfg); 
    g_autotune_active = true; 
    if (auto d = abbot::motor::getActiveMotorDriver()) d->enableMotors();
}
void stopAutotune() { g_autotune_engine.stop(); g_autotune_active = false; }
bool isAutotuning() { return g_autotune_active; }
const char* getAutotuneStatus() { return g_autotune_active ? "RUNNING" : "IDLE"; }
void applyAutotuneGains() { 
    const auto& res = g_autotune_engine.getResult();
    if (res.success) setGains(res.Kp * (180.0f/M_PI), res.Ki * (180.0f/M_PI), res.Kd * (180.0f/M_PI));
}

void setAutotuneRelay(float a) { g_autotune_cfg.relay_amplitude = a; }
void setAutotuneDeadband(float d) { g_autotune_cfg.deadband = d; }
void setAutotuneMaxAngle(float m) { g_autotune_cfg.max_pitch_abort = m; }

void setMotorGains(float l, float r) {
    g_left_motor_scale = l; g_right_motor_scale = r;
    if (g_prefs_ready) { g_preferences_helper.putFloat("mg_L", l); g_preferences_helper.putFloat("mg_R", r); }
}
void getMotorGains(float &l, float &r) { l = g_left_motor_scale; r = g_right_motor_scale; }

void setMode(ControllerMode m) { 
    abbot::balancing::BalancingManager::getInstance().setStrategy((abbot::balancing::StrategyType)m); 
    if (g_prefs_ready) {
        g_preferences_helper.putInt("mode", (int)m);
    }
}
ControllerMode getMode() { return (ControllerMode)abbot::balancing::BalancingManager::getInstance().getActiveType(); }

void setCascadedGains(const CascadedGains &g) {
    auto* s = abbot::balancing::BalancingManager::getInstance().getStrategy(abbot::balancing::StrategyType::CASCADED_LQR);
    if (s) {
        auto* lqr = static_cast<abbot::balancing::CascadedLqrStrategy*>(s);
        auto cfg = lqr->getConfig();
        cfg.k_pitch = g.k_pitch; cfg.k_gyro = g.k_gyro; cfg.k_dist = g.k_dist; cfg.k_speed = g.k_speed;
        lqr->setConfig(cfg);
    }
}
void getCascadedGains(CascadedGains &g) {
    auto* s = abbot::balancing::BalancingManager::getInstance().getStrategy(abbot::balancing::StrategyType::CASCADED_LQR);
    if (s) {
        auto cfg = static_cast<abbot::balancing::CascadedLqrStrategy*>(s)->getConfig();
        g.k_pitch = cfg.k_pitch; g.k_gyro = cfg.k_gyro; g.k_dist = cfg.k_dist; g.k_speed = cfg.k_speed;
    }
}

void setAdaptiveTrimEnabled(bool e) {
    auto* s = abbot::balancing::BalancingManager::getInstance().getStrategy(abbot::balancing::StrategyType::CASCADED_LQR);
    if (s) {
        auto* lqr = static_cast<abbot::balancing::CascadedLqrStrategy*>(s);
        auto cfg = lqr->getConfig();
        cfg.adaptive_trim_enabled = e;
        lqr->setConfig(cfg);
    }
}
bool isAdaptiveTrimEnabled() {
    auto* s = abbot::balancing::BalancingManager::getInstance().getStrategy(abbot::balancing::StrategyType::CASCADED_LQR);
    return s ? static_cast<abbot::balancing::CascadedLqrStrategy*>(s)->getConfig().adaptive_trim_enabled : false;
}

void calibrateTrim() { 
    // Simplified: uses current pitch as reference if you hold it stable
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "Trim calibration stub"); 
}
void showTrim() { LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "Trim show stub"); }
void resetTrim() { LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "Trim reset stub"); }
void calibrateStartThresholds() { LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "Start calibration stub"); }
void setAdaptiveStart(bool e) { g_adaptive_start_active = e; }
bool getAdaptiveStart() { return g_adaptive_start_active; }
void calibrateDeadband() { LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "Deadband calibration stub"); }

} // namespace controller
} // namespace balancer
} // namespace abbot
