#include "balancer_controller.h"

// Implementation of the runtime controller that owns state, persistence and
// motor commanding. This was split out from the PID implementation to keep
// SystemTasks and the PID algorithm decoupled.

#include "autotune_controller.h"
#include "logging.h"
#include "imu_drivers/imu_manager.h"
#include "motor_drivers/driver_manager.h"
#include "pid_controller.h"
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

static PIDController g_pid;
static bool g_active = false;
static float g_last_cmd = 0.0f;
static const float g_cmd_slew = BALANCER_CMD_SLEW_LIMIT;
static float g_deadband = BALANCER_MOTOR_MIN_OUTPUT;
static float g_min_cmd = BALANCER_MIN_CMD; // Valeur minimale configurable appliquée hors deadband
static float g_pitch_trim_rad = 0.0f;
static float g_calibrated_trim_rad = 0.0f;  // Calibrated trim from NVS (absolute 0° reference)
static bool g_has_calibrated_trim = false;  // True if calibrated trim was loaded from NVS
static Preferences g_prefs;
static bool g_prefs_started = false;
static AutotuneController g_autotune;
static bool g_autotune_active = false;
static AutotuneController::Config g_autocfg; // configurable autotune params
// Motor gain scaling (for asymmetric motor compensation)
static float g_left_motor_gain = BALANCER_LEFT_MOTOR_GAIN;
static float g_right_motor_gain = BALANCER_RIGHT_MOTOR_GAIN;
// Adaptive start thresholds
static bool g_adaptive_start_enabled = false;
static float g_start_threshold_left = 0.0f;
static float g_start_threshold_right = 0.0f;
// Latest IMU sample (robot frame)
static float g_last_accel[3] = {0.0f, 0.0f, 0.0f};
static float g_last_gyro[3] = {0.0f, 0.0f, 0.0f}; // rad/s

// Encoder velocity tracking for balancer (read at PID loop rate)
// Provides high-frequency velocity feedback for future cascaded control
struct EncoderVelocityState {
  int32_t last_encoder_left = 0;
  int32_t last_encoder_right = 0;
  uint32_t last_read_time_us = 0;
  float velocity_left_counts_per_sec = 0.0f;
  float velocity_right_counts_per_sec = 0.0f;
  bool initialized = false;
};

static EncoderVelocityState g_encoder_velocity_state;
static SemaphoreHandle_t g_encoder_mutex = nullptr;
static TaskHandle_t g_telemetry_task_handle = nullptr;

// Forward declarations
static void updateEncoderVelocity(float dt_sec);

/**
 * @brief Background task for motor telemetry.
 * Decouples slow RS485 encoder reads from the fast PID loop.
 */
static void telemetryTask(void *pvParameters) {
    const TickType_t xDelay = (BALANCER_ENCODER_UPDATE_MS > 0) ? pdMS_TO_TICKS(BALANCER_ENCODER_UPDATE_MS) : pdMS_TO_TICKS(100);
    while (true) {
        // We poll encoders regardless of balancer activity to keep odom fresh,
        // but only if a driver is available.
        updateEncoderVelocity(0.0f);
        vTaskDelay(xDelay);
    }
}

// Drive setpoints (high-level): normalized forward and turn commands
static float g_drive_target_v = 0.0f; // desired normalized forward [-1..1]
static float g_drive_target_w = 0.0f; // desired normalized turn [-1..1]
static float g_drive_v_filtered = 0.0f;
static float g_drive_last_pitch_setpoint_deg = 0.0f; // last pitch setpoint in degrees
static int g_last_cmd_sign = 0; // for direction change detection
// Configuration for drive->pitch mapping (from drive_config.h)
static float g_drive_max_pitch_rad =
    degToRad(DRIVE_MAX_PITCH_DEG); // configured max pitch (deg)
static const float g_drive_v_slew =
    DRIVE_V_SLEW; // units per second (configured)
// Small non-blocking delay (ms) before enabling motors after a START
static const uint32_t g_enable_delay_ms = 250;
static uint32_t g_pending_enable_ts = 0;
static const float g_start_stable_angle_rad =
    degToRad(BALANCER_START_STABLE_ANGLE_DEG);
static const float g_start_stable_rate_rad_s =
    degToRad(BALANCER_START_STABLE_PITCH_RATE_DEG_S);
static const float g_fall_stop_angle_rad =
    degToRad(BALANCER_FALL_STOP_ANGLE_DEG);
static const float g_fall_stop_rate_rad_s =
    degToRad(BALANCER_FALL_STOP_RATE_DEG_S);
static const float g_trim_max_rad = degToRad(BALANCER_TRIM_MAX_DEG);

void init() {
  if (g_encoder_mutex == nullptr) {
    g_encoder_mutex = xSemaphoreCreateMutex();
  }
  if (g_telemetry_task_handle == nullptr) {
#if BALANCER_ENABLE_ENCODER_UPDATES
    xTaskCreatePinnedToCore(telemetryTask, "telemetryTask", 4096, nullptr, 1, &g_telemetry_task_handle, 0); // Core 0 to avoid IMU core (1)
#endif
  }

  g_pid.begin(BALANCER_DEFAULT_KP, BALANCER_DEFAULT_KI, BALANCER_DEFAULT_KD,
              BALANCER_INTEGRATOR_LIMIT);
  g_pid.setLeakCoeff(BALANCER_INTEGRATOR_LEAK_COEFF);
  g_last_cmd = 0.0f;
  if (g_prefs.begin("abbot", false)) {
    g_prefs_started = true;
    char key[32];
    const char *fname = abbot::filter::getCurrentFilterName();
    // NVS keys are limited in length (typically 15 bytes). Build a safe
    // key by truncating the filter name if necessary so that suffixes like
    // "_bp" fit within limits.
    auto makePrefKey = [](char *out, size_t outsz, const char *fname,
                          const char *suffix) {
      const size_t max_nvs_key = 15; // NVS key length limit
      size_t suf = strlen(suffix);
      size_t max_name = (suf >= max_nvs_key) ? 0 : (max_nvs_key - suf);
      // copy up to max_name characters from fname
      size_t copylen = strlen(fname);
      if (copylen > max_name) {
        copylen = max_name;
      }
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
    // Load persisted minimum command applied outside deadband
    g_min_cmd = g_prefs.getFloat("min_cmd", g_min_cmd);
    // Load calibrated pitch trim from NVS (absolute 0° reference)
    g_calibrated_trim_rad = g_prefs.getFloat("trim_cal", 0.0f);
    g_has_calibrated_trim = g_prefs.getBool("trim_ok", false);
    // Load adaptive start settings
    g_adaptive_start_enabled = g_prefs.getBool("adaptive_st", false);
    g_start_threshold_left = g_prefs.getFloat("st_L", 0.0f);
    g_start_threshold_right = g_prefs.getFloat("st_R", 0.0f);
    // Load motor gains
    g_left_motor_gain = g_prefs.getFloat("mg_L", BALANCER_LEFT_MOTOR_GAIN);
    g_right_motor_gain = g_prefs.getFloat("mg_R", BALANCER_RIGHT_MOTOR_GAIN);

    char buf[128];
    snprintf(buf, sizeof(buf),
         "BALANCER: controller initialized (Kp=%.4f Ki=%.4f Kd=%.4f "
         "deadband=%.4f min_cmd=%.4f adaptive=%s gains=%.2f/%.2f)",
         (double)bp, (double)bi, (double)bd, (double)g_deadband, (double)g_min_cmd,
         g_adaptive_start_enabled ? "ON" : "OFF", (double)g_left_motor_gain, (double)g_right_motor_gain);
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, buf);
    if (g_has_calibrated_trim) {
      LOG_PRINTF(abbot::log::CHANNEL_DEFAULT,
                 "BALANCER: calibrated trim loaded = %.3f deg\n",
                 (double)radToDeg(g_calibrated_trim_rad));
    } else {
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT,
                  "BALANCER: no calibrated trim - will use dynamic capture");
    }
  } else {
    g_prefs_started = false;
    char buf[128];
    snprintf(buf, sizeof(buf),
         "BALANCER: controller initialized (defaults used) - Kp=%.4f "
         "Ki=%.4f Kd=%.4f deadband=%.4f min_cmd=%.4f",
         (double)BALANCER_DEFAULT_KP, (double)BALANCER_DEFAULT_KI,
         (double)BALANCER_DEFAULT_KD, (double)g_deadband, (double)g_min_cmd);
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, buf);
  }
}

void setDriveSetpoints(float v_norm, float w_norm) {
  // Apply joystick deadzone to avoid drift when stick is released
  if (fabsf(v_norm) < DRIVE_JOYSTICK_DEADZONE) {
    v_norm = 0.0f;
  }
  if (fabsf(w_norm) < DRIVE_JOYSTICK_DEADZONE) {
    w_norm = 0.0f;
  }

  // clamp inputs to [-1,1]
  if (v_norm > 1.0f) {
    v_norm = 1.0f;
  }
  if (v_norm < -1.0f) {
    v_norm = -1.0f;
  }
  if (w_norm > 1.0f) {
    w_norm = 1.0f;
  }
  if (w_norm < -1.0f) {
    w_norm = -1.0f;
  }
  g_drive_target_v = v_norm;
  g_drive_target_w = w_norm;

  // If joystick is released, reset integrator to prevent "unwinding" drift
  if (v_norm == 0.0f && w_norm == 0.0f) {
    g_pid.resetIntegrator();
  }

  // Log the requested setpoints for debugging
  LOG_PRINTF(abbot::log::CHANNEL_BALANCER, "SETDRIVE: t=%lums v_req=%.3f w_req=%.3f\n",
             (unsigned long)millis(), (double)g_drive_target_v, (double)g_drive_target_w);
}

void start(float fused_pitch_rad) {
  if (g_active) {
    return;
  }
  g_active = true;
  abbot::log::enableChannel(abbot::log::CHANNEL_BALANCER);
  
  // Use calibrated trim if available (absolute 0° reference from NVS),
  // otherwise fall back to dynamic capture from current pitch.
  if (g_has_calibrated_trim) {
    g_pitch_trim_rad = g_calibrated_trim_rad;
    LOG_PRINTF(abbot::log::CHANNEL_DEFAULT,
               "BALANCER: using calibrated trim = %.3f deg\n",
               (double)radToDeg(g_pitch_trim_rad));
  } else {
    // Dynamic capture: clamp to avoid masking large errors or upside-down starts
    if (fabsf(fused_pitch_rad) > g_trim_max_rad) {
      g_pitch_trim_rad = copysignf(g_trim_max_rad, fused_pitch_rad);
    } else {
      g_pitch_trim_rad = fused_pitch_rad;
    }
    LOG_PRINTF(abbot::log::CHANNEL_DEFAULT,
               "BALANCER: dynamic trim captured = %.3f deg (no calibration)\n",
               (double)radToDeg(g_pitch_trim_rad));
  }
  
  // Defer motor enabling briefly and only enable when fusion reports ready
  // and the robot is within the auto-enable angle. This avoids enabling
  // motors while the fusion/filter state is still settling after a stop
  // or other transient which can lead to immediate undesired motion.
  float cur_pitch_deg = radToDeg(fused_pitch_rad);
  if (fabsf(cur_pitch_deg) <= BALANCER_AUTO_ENABLE_ANGLE_DEG) {
    g_pending_enable_ts = millis() + g_enable_delay_ms;
    char msg[128];
    snprintf(msg, sizeof(msg),
             "BALANCER: start requested - delaying motor enable %lums "
             "(pitch=%.2fdeg)",
             (unsigned long)g_enable_delay_ms, (double)cur_pitch_deg);
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, msg);
  } else {
    g_pending_enable_ts = 0;
    char msg[128];
    snprintf(msg, sizeof(msg),
             "BALANCER: NOT enabling motors - pitch %.2f deg > limit %.1f deg",
             (double)cur_pitch_deg, (double)BALANCER_AUTO_ENABLE_ANGLE_DEG);
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, msg);
  }
  g_pid.reset();
  g_last_cmd = 0.0f;
  
  // Reinitialize the filter from current accelerometer to instantly match
  // the actual robot orientation. With low beta (0.033), the filter takes
  // too long to converge if the robot was moved while stopped. This ensures
  // the filter state matches reality when we START balancing.
  abbot::reinitFilterFromAccel();
  
  // Reset drive setpoint state on start to avoid carrying over joystick
  // commands or previous pitch setpoints which can cause the robot to
  // request a non-zero pitch immediately after re-enabling.
  g_drive_target_v = 0.0f;
  g_drive_target_w = 0.0f;
  g_drive_v_filtered = 0.0f;
  g_drive_last_pitch_setpoint_deg = 0.0f;
  g_last_cmd_sign = 0;
  char buf[160];
  {
    bool men = false;
    if (auto driver = abbot::motor::getActiveMotorDriver()) {
      men = driver->areMotorsEnabled();
    }
    const char* filter_name = abbot::filter::getCurrentFilterName();
    snprintf(buf, sizeof(buf), "BALANCER: started [Filter: %s] (Kp=%.4f Ki=%.4f Kd=%.4f) - motors %s",
             filter_name, g_pid.getKp(), g_pid.getKi(), g_pid.getKd(),
             men ? "ENABLED" : "NOT ENABLED");
  }
  LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, buf);
  
  // Debug output of configuration state
  LOG_PRINTF(abbot::log::CHANNEL_DEFAULT, "BALANCER: encoders_enabled=%d log_interval=%dms\n", 
             (int)BALANCER_ENABLE_ENCODER_UPDATES, (int)BALANCER_DEBUG_LOG_INTERVAL_MS);
}

void stop() {
  if (!g_active) {
    return;
  }
  g_active = false;
  g_pitch_trim_rad = 0.0f;
  abbot::log::disableChannel(abbot::log::CHANNEL_BALANCER);
  if (auto driver = abbot::motor::getActiveMotorDriver()) {
    driver->setMotorCommandBoth(0.0f, 0.0f);
    driver->disableMotors();
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
  g_drive_last_pitch_setpoint_deg = 0.0f;
  g_last_cmd_sign = 0;
  LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT,
              "BALANCER: stopped - motors DISABLED");
}

bool isActive() {
  return g_active;
}

void setGains(float kp, float ki, float kd) {
  g_pid.setGains(kp, ki, kd);
  if (g_prefs_started) {
    char key[32];
    const char *fname = abbot::filter::getCurrentFilterName();
    auto makePrefKey = [](char *out, size_t outsz, const char *fname,
                          const char *suffix) {
      const size_t max_nvs_key = 15; // NVS key length limit
      size_t suf = strlen(suffix);
      size_t max_name = (suf >= max_nvs_key) ? 0 : (max_nvs_key - suf);
      size_t copylen = strlen(fname);
      if (copylen > max_name) {
        copylen = max_name;
      }
      if (copylen > 0) {
        memcpy(out, fname, copylen);
        out[copylen] = '\0';
      } else {
        out[0] = '\0';
      }
      strncat(out, suffix, outsz - strlen(out) - 1);
    };
    makePrefKey(key, sizeof(key), fname, "_bp");
    g_prefs.putFloat(key, kp);
    makePrefKey(key, sizeof(key), fname, "_bi");
    g_prefs.putFloat(key, ki);
    makePrefKey(key, sizeof(key), fname, "_bd");
    g_prefs.putFloat(key, kd);
  }
  char buf[128];
  snprintf(buf, sizeof(buf),
           "BALANCER: gains set Kp=%.4f Ki=%.4f Kd=%.4f (persisted=%s)",
           (double)kp, (double)ki, (double)kd, g_prefs_started ? "yes" : "no");
  LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, buf);
}

void getGains(float &kp, float &ki, float &kd) {
  // Read persisted values if available, otherwise defaults
  if (g_prefs_started) {
    char key[32];
    const char *fname = abbot::filter::getCurrentFilterName();
    auto makePrefKey = [](char *out, size_t outsz, const char *fname,
                          const char *suffix) {
      const size_t max_nvs_key = 15; // NVS key length limit
      size_t suf = strlen(suffix);
      size_t max_name = (suf >= max_nvs_key) ? 0 : (max_nvs_key - suf);
      size_t copylen = strlen(fname);
      if (copylen > max_name) {
        copylen = max_name;
      }
      if (copylen > 0) {
        memcpy(out, fname, copylen);
        out[copylen] = '\0';
      } else {
        out[0] = '\0';
      }
      strncat(out, suffix, outsz - strlen(out) - 1);
    };
    makePrefKey(key, sizeof(key), fname, "_bp");
    kp = g_prefs.getFloat(key, BALANCER_DEFAULT_KP);
    makePrefKey(key, sizeof(key), fname, "_bi");
    ki = g_prefs.getFloat(key, BALANCER_DEFAULT_KI);
    makePrefKey(key, sizeof(key), fname, "_bd");
    kd = g_prefs.getFloat(key, BALANCER_DEFAULT_KD);
  } else {
    kp = BALANCER_DEFAULT_KP;
    ki = BALANCER_DEFAULT_KI;
    kd = BALANCER_DEFAULT_KD;
  }
}

void resetGainsToDefaults() {
  g_pid.setGains(BALANCER_DEFAULT_KP, BALANCER_DEFAULT_KI, BALANCER_DEFAULT_KD);
  if (g_prefs_started) {
    char key[32];
    const char *fname = abbot::filter::getCurrentFilterName();
    auto makePrefKey = [](char *out, size_t outsz, const char *fname,
                          const char *suffix) {
      const size_t max_nvs_key = 15; // NVS key length limit
      size_t suf = strlen(suffix);
      size_t max_name = (suf >= max_nvs_key) ? 0 : (max_nvs_key - suf);
      size_t copylen = strlen(fname);
      if (copylen > max_name) {
        copylen = max_name;
      }
      if (copylen > 0) {
        memcpy(out, fname, copylen);
        out[copylen] = '\0';
      } else {
        out[0] = '\0';
      }
      strncat(out, suffix, outsz - strlen(out) - 1);
    };
    makePrefKey(key, sizeof(key), fname, "_bp");
    g_prefs.putFloat(key, BALANCER_DEFAULT_KP);
    makePrefKey(key, sizeof(key), fname, "_bi");
    g_prefs.putFloat(key, BALANCER_DEFAULT_KI);
    makePrefKey(key, sizeof(key), fname, "_bd");
    g_prefs.putFloat(key, BALANCER_DEFAULT_KD);
    // preserve legacy global deadband key
    g_prefs.putFloat("db", BALANCER_MOTOR_MIN_OUTPUT);
  }
  g_deadband = BALANCER_MOTOR_MIN_OUTPUT;
  char buf[128];
  snprintf(buf, sizeof(buf),
           "BALANCER: reset to defaults Kp=%.4f Ki=%.4f Kd=%.4f deadband=%.4f",
           (double)BALANCER_DEFAULT_KP, (double)BALANCER_DEFAULT_KI,
           (double)BALANCER_DEFAULT_KD, (double)BALANCER_MOTOR_MIN_OUTPUT);
  LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, buf);
}

void setDeadband(float db) {
  g_deadband = db;
  if (g_prefs_started) {
    g_prefs.putFloat("db", g_deadband);
  }
  char buf[128];
  snprintf(buf, sizeof(buf), "BALANCER: deadband set to %.6f (persisted=%s)",
           (double)g_deadband, g_prefs_started ? "yes" : "no");
  LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, buf);
}

float getDeadband() {
  return g_deadband;
}

void calibrateDeadband() {
  LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT,
              "DEADBAND calibration not yet implemented. Use 'BALANCE DEADBAND "
              "SET <v>' to store a value.");
}

// Setter/Getter pour min_cmd
void setMinCmd(float min_cmd) {
  g_min_cmd = min_cmd;
  if (g_prefs_started) {
    g_prefs.putFloat("min_cmd", g_min_cmd);
  }
  char buf[64];
  snprintf(buf, sizeof(buf), "BALANCER: min_cmd set to %.6f (persisted=%s)",
           (double)g_min_cmd, g_prefs_started ? "yes" : "no");
  LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, buf);
}

float getMinCmd() {
  return g_min_cmd;
}

void setAdaptiveStart(bool enabled) {
  g_adaptive_start_enabled = enabled;
  if (g_prefs_started) {
    g_prefs.putBool("adaptive_st", g_adaptive_start_enabled);
  }
  LOG_PRINTF(abbot::log::CHANNEL_DEFAULT, "BALANCER: adaptive start %s\n",
             g_adaptive_start_enabled ? "ENABLED" : "DISABLED");
}

bool getAdaptiveStart() {
  return g_adaptive_start_enabled;
}

void calibrateStartThresholds() {
  bool was_active = g_active;
  g_active = false; // Disable PID

  LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT,
              "BALANCER: Starting start threshold calibration...");
  LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT,
              "BALANCER: Ensure robot is on a support where wheels can spin "
              "freely.");

  float thresholds_L[2] = {0.0f, 0.0f}; // Fwd, Rev
  float thresholds_R[2] = {0.0f, 0.0f}; // Fwd, Rev

  auto calibrateSide = [](abbot::motor::IMotorDriver::MotorSide side, float &fwd_st,
                          float &rev_st) {
    const float step = 0.005f;
    const uint32_t step_ms = 300; 
    const int32_t tick_threshold = 50; // ~7.3 degrees, clearly visible

    const char* side_name = (side == abbot::motor::IMotorDriver::MotorSide::LEFT) ? "LEFT" : "RIGHT";
    LOG_PRINTF(abbot::log::CHANNEL_DEFAULT, "BALANCER: Calibrating %s side...\n", side_name);

    // Forward
    fwd_st = 0.5f;
    int32_t start_enc = abbot::motor::readEncoderBySide(side);
    for (float cmd = 0.0f; cmd <= 0.5f; cmd += step) {
      abbot::motor::setMotorCommandBySide(side, cmd);
      vTaskDelay(pdMS_TO_TICKS(step_ms));
      int32_t current_enc = abbot::motor::readEncoderBySide(side);
      int32_t diff = abs(current_enc - start_enc);
      
      if (cmd > 0.01f && ((int)(cmd * 1000) % 20 == 0)) { // Log every 0.02 step to show progress
          LOG_PRINTF(abbot::log::CHANNEL_DEFAULT, "  %s Fwd: cmd=%.3f, ticks=%ld\n", side_name, (double)cmd, (long)diff);
      }

      if (diff >= tick_threshold) {
        fwd_st = cmd;
        LOG_PRINTF(abbot::log::CHANNEL_DEFAULT, "  => %s Fwd Detected: %.4f (at %ld ticks)\n", 
                   side_name, (double)fwd_st, (long)diff);
        break;
      }
    }
    abbot::motor::setMotorCommandBySide(side, 0.0f);
    vTaskDelay(pdMS_TO_TICKS(500));

    // Reverse
    rev_st = 0.5f;
    start_enc = abbot::motor::readEncoderBySide(side);
    for (float cmd = 0.0f; cmd >= -0.5f; cmd -= step) {
      abbot::motor::setMotorCommandBySide(side, cmd);
      vTaskDelay(pdMS_TO_TICKS(step_ms));
      int32_t current_enc = abbot::motor::readEncoderBySide(side);
      int32_t diff = abs(current_enc - start_enc);

      if (fabsf(cmd) > 0.01f && ((int)(fabsf(cmd) * 1000) % 20 == 0)) {
          LOG_PRINTF(abbot::log::CHANNEL_DEFAULT, "  %s Rev: cmd=%.3f, ticks=%ld\n", side_name, (double)cmd, (long)diff);
      }

      if (diff >= tick_threshold) {
        rev_st = fabsf(cmd);
        LOG_PRINTF(abbot::log::CHANNEL_DEFAULT, "  => %s Rev Detected: %.4f (at %ld ticks)\n", 
                   side_name, (double)rev_st, (long)diff);
        break;
      }
    }
    abbot::motor::setMotorCommandBySide(side, 0.0f);
    vTaskDelay(pdMS_TO_TICKS(500));
  };

  calibrateSide(abbot::motor::IMotorDriver::MotorSide::LEFT, thresholds_L[0],
                thresholds_L[1]);
  calibrateSide(abbot::motor::IMotorDriver::MotorSide::RIGHT, thresholds_R[0],
                thresholds_R[1]);

  g_start_threshold_left = (thresholds_L[0] + thresholds_L[1]) / 2.0f;
  g_start_threshold_right = (thresholds_R[0] + thresholds_R[1]) / 2.0f;

  if (g_prefs_started) {
    g_prefs.putFloat("st_L", g_start_threshold_left);
    g_prefs.putFloat("st_R", g_start_threshold_right);
  }

  LOG_PRINTF(abbot::log::CHANNEL_DEFAULT,
             "BALANCER: Calibration done. L=%.4f, R=%.4f\n",
             (double)g_start_threshold_left, (double)g_start_threshold_right);

  g_active = was_active;
}

void setLatestImuSample(const float accel_robot[3], const float gyro_robot[3]) {
  if (accel_robot) {
    g_last_accel[0] = accel_robot[0];
    g_last_accel[1] = accel_robot[1];
    g_last_accel[2] = accel_robot[2];
  }
  if (gyro_robot) {
    g_last_gyro[0] = gyro_robot[0];
    g_last_gyro[1] = gyro_robot[1];
    g_last_gyro[2] = gyro_robot[2];
  }
}

// Helper: apply drive->pitch mapping with slew limiting and compute PID output
// Returns the raw PID controller output (not yet slew/ deadband/clamped for
// motor)
// NOTE: fused_pitch and fused_pitch_rate are now expected in DEGREES
static float computePidWithDriveSetpoint(float fused_pitch_deg,
                                         float fused_pitch_rate_deg_s, float dt) {
  // Filter normalized v toward target with simple first-order slew limiter
  if (g_drive_v_filtered != g_drive_target_v) {
    float max_dv = g_drive_v_slew * dt;
    float dv = g_drive_target_v - g_drive_v_filtered;
    if (dv > max_dv) {
      dv = max_dv;
    }
    if (dv < -max_dv) {
      dv = -max_dv;
    }
    g_drive_v_filtered += dv;
  }
  // pitch setpoint from filtered velocity command (convert to degrees)
  float pitch_setpoint_rad = g_drive_v_filtered * g_drive_max_pitch_rad;
  // clamp pitch setpoint (in radians before conversion)
  if (pitch_setpoint_rad > g_drive_max_pitch_rad) {
    pitch_setpoint_rad = g_drive_max_pitch_rad;
  }
  if (pitch_setpoint_rad < -g_drive_max_pitch_rad) {
    pitch_setpoint_rad = -g_drive_max_pitch_rad;
  }
  
  float pitch_setpoint_deg = radToDeg(pitch_setpoint_rad);
  
  // --- Safety: Setpoint Attenuation ---
  // If the robot is far from vertical (large error), reduce the drive setpoint
  // to prioritize stabilization. This prevents the "fighting" feeling where
  // a large drive setpoint consumes all motor authority.
  float current_error_deg = fused_pitch_deg - 0.0f; // error relative to vertical
  if (fabsf(current_error_deg) > DRIVE_SAFETY_ATTENUATION_START_DEG) {
    // Gradually reduce setpoint as error grows from start to end degrees
    float range = DRIVE_SAFETY_ATTENUATION_END_DEG - DRIVE_SAFETY_ATTENUATION_START_DEG;
    float attenuation = 1.0f - (fabsf(current_error_deg) - DRIVE_SAFETY_ATTENUATION_START_DEG) / range;
    if (attenuation < 0.0f) {
      attenuation = 0.0f;
    }
    pitch_setpoint_deg *= attenuation;
  }

  // estimate pitch_setpoint rate (in deg/s)
  float pitch_setpoint_rate_deg_s = 0.0f;
  if (dt > 0.0f) {
    pitch_setpoint_rate_deg_s = (pitch_setpoint_deg - g_drive_last_pitch_setpoint_deg) / dt;
  }
  g_drive_last_pitch_setpoint_deg = pitch_setpoint_deg;

  // PID input: error = measurement - setpoint (so positive error commands forward)
  // When robot leans forward (positive pitch), error is positive → command forward to correct
  float pid_in_deg = fused_pitch_deg - pitch_setpoint_deg;
  float pid_rate_deg_s = fused_pitch_rate_deg_s - pitch_setpoint_rate_deg_s;
  float pid_out = g_pid.update(pid_in_deg, pid_rate_deg_s, dt);

  // Rate-limited debug logging to avoid spamming serial
  static uint32_t last_dbg_ms = 0;
  uint32_t now_ms = millis();
  if (now_ms - last_dbg_ms > 200) {
    LOG_PRINTF(abbot::log::CHANNEL_BALANCER,
               "DRIVE DBG t=%lums tgtV=%.3f filtV=%.3f pitch_setpoint=%.3fdeg "
               "pitch_setpoint_rate=%.3fdeg/s pid_in=%.3fdeg pid_rate=%.3fdeg/s "
               "pid_out=%.3f\n",
               (unsigned long)now_ms, (double)g_drive_target_v,
               (double)g_drive_v_filtered, (double)pitch_setpoint_deg,
               (double)pitch_setpoint_rate_deg_s, (double)pid_in_deg,
               (double)pid_rate_deg_s, (double)pid_out);
    last_dbg_ms = now_ms;
  }

  return pid_out;
}

// Read encoders and compute velocity at background task rate
static void updateEncoderVelocity(float /*dt_sec*/) {
  auto active_driver = abbot::motor::getActiveMotorDriver();
  if (!active_driver) {
    return;
  }
  
  uint32_t now_us = (uint32_t)esp_timer_get_time();
  int32_t encoder_left = 0;
  int32_t encoder_right = 0;
  
  // Use the optimized parallel read 
  active_driver->readEncodersBoth(encoder_left, encoder_right);
  
  if (g_encoder_mutex && xSemaphoreTake(g_encoder_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    if (g_encoder_velocity_state.initialized) {
      uint32_t dt_us = now_us - g_encoder_velocity_state.last_read_time_us;
      if (dt_us > 0) {
        float dt_sec_actual = (float)dt_us / 1000000.0f;
        
        int32_t delta_left = encoder_left - g_encoder_velocity_state.last_encoder_left;
        int32_t delta_right = encoder_right - g_encoder_velocity_state.last_encoder_right;
        
        // Velocity in counts/sec
        g_encoder_velocity_state.velocity_left_counts_per_sec = 
            (float)delta_left / dt_sec_actual;
        g_encoder_velocity_state.velocity_right_counts_per_sec = 
            (float)delta_right / dt_sec_actual;
      }
    }
    
    g_encoder_velocity_state.last_encoder_left = encoder_left;
    g_encoder_velocity_state.last_encoder_right = encoder_right;
    g_encoder_velocity_state.last_read_time_us = now_us;
    g_encoder_velocity_state.initialized = true;
    xSemaphoreGive(g_encoder_mutex);
  }
}

// Called each IMU loop to compute and (if active) command motors. Returns
// the computed command (normalized) regardless of whether it was sent.
// Note: Pitch sign is now handled by axis mapping in FusionConfig
// (accel_sign/gyro_sign), so pitch arrives with correct sign: positive = tilted
// forward.
float processCycle(float fused_pitch, float last_fused_pitch_rate, float dt) {
  // 1. Filter the raw gyro pitch rate to reduce D-term noise/jitter
  static float filtered_pitch_rate = 0.0f;
  const float alpha = BALANCER_PITCH_RATE_ALPHA;
  filtered_pitch_rate = (alpha * last_fused_pitch_rate) + ((1.0f - alpha) * filtered_pitch_rate);
  
  float fused_pitch_rate = filtered_pitch_rate;

  // Encoder updates are now handled by a background task to minimize PID loop latency.
  
  // Check if autotuning is active
  if (g_autotune_active && g_autotune.isActive()) {
    // Pitch is already correctly signed from fusion
    float error_rad = fused_pitch;
    float error_deg = radToDeg(error_rad);

    // Pass dt in seconds for high-precision timing inside autotune
    float autotune_raw = g_autotune.update(error_deg, dt);
    float autotune_cmd = autotune_raw; // No inversion - consistent with PID

    // Debug: log autotune state and command
    static uint32_t last_log_ms = 0;
    uint32_t now_ms = millis();
    if (now_ms - last_log_ms > 500) {
      char dbg[128];
      int men = 0;
      if (auto driver = abbot::motor::getActiveMotorDriver()) {
        men = driver->areMotorsEnabled() ? 1 : 0;
      }
      snprintf(dbg, sizeof(dbg),
               "AUTOTUNE: state=%d err=%.2f° cmd=%.3f motors_en=%d",
               (int)g_autotune.getState(), (double)error_deg,
               (double)autotune_cmd, men);
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, dbg);
      last_log_ms = now_ms;
    }

    // Check if autotune just finished
    if (!g_autotune.isActive()) {
      g_autotune_active = false;

      if (g_autotune.getState() == AutotuneController::State::COMPLETE) {
        LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT,
                    "AUTOTUNE: COMPLETE - use 'AUTOTUNE APPLY' to set gains");
      } else if (g_autotune.getState() == AutotuneController::State::FAILED) {
        char buf[128];
        snprintf(buf, sizeof(buf), "AUTOTUNE: FAILED - %s",
                 g_autotune.getFailureReason());
        LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, buf);
      }

      // Zero and disable motors
      if (auto driver = abbot::motor::getActiveMotorDriver()) {
        driver->setMotorCommandBoth(0.0f, 0.0f);
        driver->disableMotors();
      }
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT,
                  "AUTOTUNE: motors disabled automatically");
      return 0.0f;
    }

    // Apply autotune command to motors (apply per-motor gain scaling)
    if (auto driver = abbot::motor::getActiveMotorDriver()) {
      if (driver->areMotorsEnabled()) {
        float left_at = autotune_cmd * g_left_motor_gain;
        float right_at = autotune_cmd * g_right_motor_gain;
        driver->setMotorCommandBoth(left_at, right_at);
      }
    }

    return autotune_cmd;
  }

  if (!g_active) {
    return 0.0f;
  }

  // Fall-detection guard: if the robot is clearly down, stop the balancer to
  // avoid fighting on the ground. Uses both angle and optional rate threshold.
  if (fabsf(fused_pitch) > g_fall_stop_angle_rad ||
      (g_fall_stop_rate_rad_s > 0.0f &&
       fabsf(fused_pitch_rate) > g_fall_stop_rate_rad_s)) {
    stop();
    LOG_PRINTF(abbot::log::CHANNEL_DEFAULT,
               "BALANCER: auto-stopped (fell) pitch=%.2fdeg rate=%.2fdeg/s",
               (double)radToDeg(fused_pitch),
               (double)radToDeg(fused_pitch_rate));
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
        LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT,
                    "BALANCER: delayed enable deferred - fusion not ready");
        g_pending_enable_ts = now + g_enable_delay_ms;
      } else if (fabsf(fused_pitch) > g_start_stable_angle_rad) {
        char msg[128];
        snprintf(msg, sizeof(msg),
                 "BALANCER: delayed enable deferred - pitch %.2f deg outside "
                 "%.2f deg",
                 (double)cur_pitch_deg,
                 (double)radToDeg(g_start_stable_angle_rad));
        LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, msg);
        g_pending_enable_ts = now + g_enable_delay_ms;
      } else if (fabsf(fused_pitch_rate) > g_start_stable_rate_rad_s) {
        char msg[128];
        snprintf(msg, sizeof(msg),
                 "BALANCER: delayed enable deferred - pitch rate %.2f deg/s > "
                 "%.2f deg/s",
                 (double)radToDeg(fused_pitch_rate),
                 (double)radToDeg(g_start_stable_rate_rad_s));
        LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, msg);
        g_pending_enable_ts = now + g_enable_delay_ms;
      } else {
        if (auto driver = abbot::motor::getActiveMotorDriver()) {
          driver->enableMotors();
        }
        LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT,
                    "BALANCER: motors ENABLED after settle");
        g_pending_enable_ts = 0;
      }
    }
  }
  // For INVERTED PENDULUM (balancing robot):
  // - Pitch > 0 (tilted forward) -> motors must go FORWARD (cmd > 0) to catch
  // - Pitch < 0 (tilted backward) -> motors must go BACKWARD (cmd < 0) to catch
  // With ay > 0 when tilted forward and our axis mapping, Madgwick produces
  // pitch that may be positive or negative for forward tilt depending on
  // convention. We use positive command = motors forward.
  //
  // NOTE: Motor directions are CORRECT (validated with MOTOR VEL commands).
  //       DO NOT change motor inversion in motor_config.h!
  //       Pitch sign is handled by accel_sign/gyro_sign in FusionConfig.h

  // Convert pitch from radians to degrees for PID (more intuitive gains)
  float pitch_for_control_deg = radToDeg(fused_pitch - g_pitch_trim_rad);
  
  // Use the filtered pitch rate for the D term to reduce high-frequency noise/jitter.
  // fused_pitch_rate is already filtered at the top of processCycle and is in rad/s.
  float pitch_rate_deg_s = radToDeg(fused_pitch_rate);
  
  // Compute PID output taking into account the drive-setpoint->pitch mapping
  float pid_out =
      computePidWithDriveSetpoint(pitch_for_control_deg, pitch_rate_deg_s, dt);
  
  // Compute pid_in here (pitch error in degrees) for logging
  // NOTE: error = measurement - setpoint (consistent with computePidWithDriveSetpoint)
  float pitch_setpoint_deg = radToDeg(g_drive_v_filtered * g_drive_max_pitch_rad);
  float pid_in_deg = pitch_for_control_deg - pitch_setpoint_deg;
  
  float cmd = pid_out; // No negation - same sign as pitch for catching behavior
  // clamp (pre-slew). Keep a copy for logging to inspect clamp vs slew behavior
  float cmd_pre_slew = cmd;
  if (cmd_pre_slew > 1.0f) {
    cmd_pre_slew = 1.0f;
  }
  if (cmd_pre_slew < -1.0f) {
    cmd_pre_slew = -1.0f;
  }
  // apply the clamp to cmd (we'll apply slew after)
  if (cmd > 1.0f) {
    cmd = 1.0f;
  }
  if (cmd < -1.0f) {
    cmd = -1.0f;
  }
  // slew (apply rate limit to prevent abrupt changes)
  float max_delta = g_cmd_slew * dt;
  float delta = cmd - g_last_cmd;
  if (delta > max_delta) {
    delta = max_delta;
  }
  if (delta < -max_delta) {
    delta = -max_delta;
  }
  cmd = g_last_cmd + delta;

  // --- Fluidity Enhancements ---
  // Note: Boosts disabled (set to 1.0 in config) for FOC linearity.
  int current_sign = (cmd > DRIVE_KICK_NOISE_FLOOR) ? 1 : ((cmd < -DRIVE_KICK_NOISE_FLOOR) ? -1 : 0);
  if (current_sign != 0 && g_last_cmd_sign != 0 && current_sign != g_last_cmd_sign) {
    cmd *= DRIVE_KICK_BOOST;
  }
  g_last_cmd_sign = current_sign;

  if (g_drive_target_v == 0.0f && fabsf(g_drive_v_filtered) > DRIVE_BRAKE_NOISE_FLOOR) {
    cmd *= DRIVE_BRAKE_BOOST;
  }

  // 3. Safety Reset: if error is huge, clear integrator to avoid violent recovery
  if (fabsf(pid_in_deg) > DRIVE_SAFETY_INTEGRATOR_RESET_DEG) {
    g_pid.resetIntegrator();
  }

  // Rate-limited debug logging for PID vs final commanded value
  // Interval controlled by BALANCER_DEBUG_LOG_INTERVAL_MS (configurable for characterization)
  static uint32_t last_cmd_dbg_ms = 0;
  uint32_t now_cmd_dbg_ms = millis();
  uint32_t log_interval = BALANCER_DEBUG_LOG_INTERVAL_MS;
  if ((log_interval == 0 || (now_cmd_dbg_ms - last_cmd_dbg_ms >= log_interval)) && 
      abbot::log::isChannelEnabled(abbot::log::CHANNEL_BALANCER)) {
    
    uint32_t bus_latency_us = 0;
    if (auto drv = abbot::motor::getActiveMotorDriver()) {
      bus_latency_us = drv->getLastBusLatencyUs();
    }
    float gx_dps = radToDeg(g_last_gyro[0]);
    float gy_dps = radToDeg(g_last_gyro[1]);
    float gz_dps = radToDeg(g_last_gyro[2]);
    
    long left_enc = 0;
    long right_enc = 0;
    // Non-blocking lock for logs: skip if busy to avoid stalling the 500Hz loop
    if (g_encoder_mutex && xSemaphoreTake(g_encoder_mutex, 0) == pdTRUE) {
      left_enc = (long)g_encoder_velocity_state.last_encoder_left;
      right_enc = (long)g_encoder_velocity_state.last_encoder_right;
      xSemaphoreGive(g_encoder_mutex);
    }

    LOG_PRINTF_TRY(
      abbot::log::CHANNEL_BALANCER,
      "BALANCER_DBG t=%lums pitch=%.2fdeg pid_in=%.3fdeg pid_out=%.3f iterm=%.4f cmd=%.3f lat=%luus ax=%.3f ay=%.3f az=%.3f gx=%.1f gy=%.1f gz=%.1f lp_hz=%.1f encL=%ld encR=%ld\n",
      (unsigned long)now_cmd_dbg_ms, (double)radToDeg(fused_pitch),
      (double)pid_in_deg, (double)pid_out, (double)(g_pid.getKi() * g_pid.getIntegrator()), (double)cmd,
      (unsigned long)bus_latency_us,
      (double)g_last_accel[0], (double)g_last_accel[1], (double)g_last_accel[2],
      (double)gx_dps, (double)gy_dps, (double)gz_dps,
      (double)(1.0f / dt),
      left_enc, right_enc);
    last_cmd_dbg_ms = now_cmd_dbg_ms;
  }
  // deadband: if command is small but non-zero, push to the edge instead of
  // zeroing so motors overcome static friction. Skip if last_cmd was zero to
  // avoid jolts at startup.
  // Nouvelle logique : si |cmd| < deadband, appliquer min_cmd (signe correct),
  // mais si |cmd| est très petit, alors cmd = 0
  float effective_min_cmd = g_min_cmd;
  if (g_adaptive_start_enabled) {
    effective_min_cmd =
        (g_start_threshold_left + g_start_threshold_right) / 2.0f;
  }

  if (fabsf(cmd) < g_deadband) {
    if (fabsf(cmd) < 0.001f) { // Hard floor reduced for FOC motors
      cmd = 0.0f;
    } else {
      float sign = (cmd > 0.0f) ? 1.0f : -1.0f;
      cmd = sign *
            ((fabsf(cmd) > effective_min_cmd) ? fabsf(cmd) : effective_min_cmd);
    }
  }
  // Charger min_cmd depuis les préférences au démarrage
  // (à placer dans la fonction d'init appropriée, ex : balancerInit ou setup)
  // g_min_cmd = g_prefs.getFloat("min_cmd", g_min_cmd);
  // if motors disabled, skip commanding but return computed value
  if (auto driver = abbot::motor::getActiveMotorDriver()) {
    if (!driver->areMotorsEnabled()) {
      static bool warned = false;
      if (!warned) {
        LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT,
                    "BALANCER: motors disabled - not commanding (use MOTOR "
                    "ENABLE to arm)");
        warned = true;
      }
      g_last_cmd = cmd;
      return cmd;
    }
    // command both motors simultaneously (apply per-motor gain scaling)
    float left_cmd = cmd * g_left_motor_gain;
    float right_cmd = cmd * g_right_motor_gain;
    driver->setMotorCommandBoth(left_cmd, right_cmd);
  } else {
    static bool warned_no_drv = false;
    if (!warned_no_drv) {
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT,
                  "BALANCER: no active motor driver - not commanding");
      warned_no_drv = true;
    }
    g_last_cmd = cmd;
    return cmd;
  }
  // LOG_PRINTF(abbot::log::CHANNEL_BALANCER, "BALANCER: cmd=%.4f err=%.6f
  // pitch_deg=%.4f\n", cmd, (double)error, (double)radToDeg(fused_pitch));
  g_last_cmd = cmd;
  return cmd;
}

void startAutotune() {
  if (g_autotune_active) {
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "AUTOTUNE: already running");
    return;
  }

  // Stop regular balancing
  if (g_active) {
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT,
                "AUTOTUNE: stopping balancer to start tuning");
    stop();
  }

  // Start with current configurable parameters (defaults in g_autocfg)
  g_autotune.start(&g_autocfg);
  g_autotune_active = true;

  // Always enable motors for autotuning
  if (auto driver = abbot::motor::getActiveMotorDriver()) {
    driver->enableMotors();
  }
  LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT,
              "AUTOTUNE: motors enabled automatically");

  LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT,
              "AUTOTUNE: started - applying relay control (HOLD THE ROBOT!)");
}

void stopAutotune() {
  if (!g_autotune_active) {
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "AUTOTUNE: not running");
    return;
  }

  g_autotune.stop();
  g_autotune_active = false;

  // Zero and disable motors
  if (auto driver = abbot::motor::getActiveMotorDriver()) {
    driver->setMotorCommand(abbot::motor::IMotorDriver::MotorSide::LEFT, 0.0f);
    driver->setMotorCommand(abbot::motor::IMotorDriver::MotorSide::RIGHT, 0.0f);
    driver->disableMotors();
  }

  LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT,
              "AUTOTUNE: stopped - motors disabled");
}

bool isAutotuning() {
  return g_autotune_active && g_autotune.isActive();
}

const char *getAutotuneStatus() {
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

void applyAutotuneGains() {
  if (g_autotune.getState() != AutotuneController::State::COMPLETE) {
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT,
                "AUTOTUNE: no successful tuning result available");
    return;
  }

  const auto &result = g_autotune.getResult();
  if (!result.success) {
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "AUTOTUNE: tuning failed");
    return;
  }

  // Apply the computed gains
  // IMPORTANT: Autotune calculates gains based on DEGREES (Input=Deg,
  // Output=0..1) But the PID controller works in RADIANS (Input=Rad,
  // Output=0..1) We must convert the gains: K_rad = K_deg * (180/PI)
  const float deg2rad_factor = 180.0f / M_PI;
  float Kp = result.Kp * deg2rad_factor;
  float Ki = result.Ki * deg2rad_factor;
  float Kd = result.Kd * deg2rad_factor;

  setGains(Kp, Ki, Kd);

  char buf[256];
  snprintf(buf, sizeof(buf),
           "AUTOTUNE: Applied gains (Rad) - Kp=%.4f Ki=%.4f Kd=%.4f | (Raw "
           "Deg: Kp=%.5f)",
           (double)Kp, (double)Ki, (double)Kd, (double)result.Kp);
  LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, buf);

  g_autotune_active = false;
}

// --- Autotune configuration setters ---
void setAutotuneRelay(float amplitude) {
  if (amplitude < 0.05f) {
    amplitude = 0.05f;
  }
  if (amplitude > 1.0f) {
    amplitude = 1.0f;
  }
  g_autocfg.relay_amplitude = amplitude;
  char buf[128];
  snprintf(buf, sizeof(buf), "AUTOTUNE: relay amplitude set to %.3f",
           (double)amplitude);
  LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, buf);
}

void setAutotuneDeadband(float deadband_deg) {
  if (deadband_deg < 0.0f) {
    deadband_deg = 0.0f;
  }
  if (deadband_deg > 10.0f) {
    deadband_deg = 10.0f;
  }
  g_autocfg.deadband = deadband_deg;
  char buf[128];
  snprintf(buf, sizeof(buf), "AUTOTUNE: deadband set to %.3f°",
           (double)deadband_deg);
  LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, buf);
}

void setAutotuneMaxAngle(float max_pitch_deg) {
  if (max_pitch_deg < 5.0f) {
    max_pitch_deg = 5.0f;
  }
  if (max_pitch_deg > 90.0f) {
    max_pitch_deg = 90.0f;
  }
  g_autocfg.max_pitch_abort = max_pitch_deg;
  char buf[128];
  snprintf(buf, sizeof(buf), "AUTOTUNE: max angle set to %.1f°",
           (double)max_pitch_deg);
  LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, buf);
}

// --- Motor gain adjustment (for asymmetric compensation) ---
void setMotorGains(float left_gain, float right_gain) {
  if (left_gain < 0.1f) {
    left_gain = 0.1f;
  }
  if (left_gain > 2.0f) {
    left_gain = 2.0f;
  }
  if (right_gain < 0.1f) {
    right_gain = 0.1f;
  }
  if (right_gain > 2.0f) {
    right_gain = 2.0f;
  }
  g_left_motor_gain = left_gain;
  g_right_motor_gain = right_gain;

  if (g_prefs_started) {
    g_prefs.putFloat("mg_L", g_left_motor_gain);
    g_prefs.putFloat("mg_R", g_right_motor_gain);
  }

  char buf[128];
  snprintf(buf, sizeof(buf), "BALANCER: motor gains set L=%.3f R=%.3f (persisted=%s)",
           (double)left_gain, (double)right_gain, g_prefs_started ? "yes" : "no");
  LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, buf);
}

void getMotorGains(float &left_gain, float &right_gain) {
  left_gain = g_left_motor_gain;
  right_gain = g_right_motor_gain;
}

// --- Calibrated trim management ---
void calibrateTrim() {
  LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "BALANCER: Starting trim calibration (stay still)...");
  
  // Reinitialize the filter from the current accelerometer reading to get in the ballpark.
  ::abbot::reinitFilterFromAccel();

  // Let filter settle briefly
  vTaskDelay(pdMS_TO_TICKS(100));

  const int N = 51; // More samples for better statistical stability (approx 1s)
  float *samples = new float[N];
  
  for (int i = 0; i < N; ++i) {
    samples[i] = ::abbot::getFusedPitch();
    // stagger samples to allow sensor noise to decorrelate
    vTaskDelay(pdMS_TO_TICKS(20));
    
    if (i % 10 == 0) {
        LOG_PRINTF(abbot::log::CHANNEL_DEFAULT, "BALANCER: Sampling trim... %d%%\n", (i * 100) / N);
    }
  }

  // compute median to reject outliers and spikes
  std::sort(samples, samples + N);
  float median = samples[N/2];
  float min_val = samples[0];
  float max_val = samples[N-1];

  delete[] samples;

  g_calibrated_trim_rad = median;
  g_has_calibrated_trim = true;

  // Persist to NVS
  if (g_prefs_started) {
    g_prefs.putFloat("trim_cal", g_calibrated_trim_rad);
    g_prefs.putBool("trim_ok", true);
  }

  float trim_deg = radToDeg(g_calibrated_trim_rad);
  float spread_deg = radToDeg(max_val - min_val);
  
  char buf[256];
  snprintf(buf, sizeof(buf),
           "BALANCER: Trim calibrated to %.4f° (spread=%.4f°), persisted=%s",
           (double)trim_deg, (double)spread_deg,
           g_prefs_started ? "yes" : "no");
  LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, buf);
}

void showTrim() {
  float calibrated_deg = radToDeg(g_calibrated_trim_rad);
  float dynamic_deg = radToDeg(g_pitch_trim_rad);
  char buf[256];
  snprintf(buf, sizeof(buf),
           "BALANCER: calibrated_trim=%.4f° (valid=%s), dynamic_trim=%.4f°, "
           "using=%s",
           (double)calibrated_deg, g_has_calibrated_trim ? "yes" : "no",
           (double)dynamic_deg,
           g_has_calibrated_trim ? "calibrated" : "dynamic");
  LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, buf);
}

void resetTrim() {
  g_calibrated_trim_rad = 0.0f;
  g_has_calibrated_trim = false;
  g_pitch_trim_rad = 0.0f;

  // Remove from NVS
  if (g_prefs_started) {
    g_prefs.remove("trim_cal");
    g_prefs.remove("trim_ok");
  }

  LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT,
              "BALANCER: calibrated trim reset (will use dynamic capture)");
}

} // namespace controller
} // namespace balancer
} // namespace abbot
