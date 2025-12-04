#include "imu_consumer_helpers.h"
#include "logging.h"
#include "tuning_capture.h"
#include "units.h"
#include "imu_calibration.h"
#include <cmath>
#include "motor_driver.h"
#include "../config/motor_config.h"
#include "../include/balancer_controller.h"

namespace abbot {
namespace imu_consumer {

float computeDt(ConsumerState &state, const IMUSample &sample, float sample_rate_hz) {
  float dt = 1.0f / sample_rate_hz;
  if (state.last_sample_timestamp_ms != 0) {
    unsigned long delta_ms = sample.ts_ms - state.last_sample_timestamp_ms;
    if (delta_ms > 0) dt = (float)delta_ms / 1000.0f;
  }
  state.last_sample_timestamp_ms = sample.ts_ms;
  return dt;
}

void accumulateWarmup(ConsumerState &state, const IMUSample &sample, float gx, float gy, float gz) {
  if (state.warmup_samples_remaining > 0) {
    state.warm_ax_sum += sample.ax;
    state.warm_ay_sum += sample.ay;
    state.warm_az_sum += sample.az;
    state.warm_gx_sum += gx;
    state.warm_gy_sum += gy;
    state.warm_gz_sum += gz;
  }
}

void finalizeWarmupIfDone(ConsumerState &state, fusion::Madgwick &madgwick, const fusion::FusionConfig &cfg) {
  if (state.warmup_samples_remaining == 0 && state.warmup_samples_total > 0) {
    float ax_avg = state.warm_ax_sum / (float)state.warmup_samples_total;
    float ay_avg = state.warm_ay_sum / (float)state.warmup_samples_total;
    float az_avg = state.warm_az_sum / (float)state.warmup_samples_total;
    state.gyro_bias[0] = state.warm_gx_sum / (float)state.warmup_samples_total;
    state.gyro_bias[1] = state.warm_gy_sum / (float)state.warmup_samples_total;
    state.gyro_bias[2] = state.warm_gz_sum / (float)state.warmup_samples_total;

    float avg_sensor_a[3] = { ax_avg, ay_avg, az_avg };
    float avg_accel_robot[3];
    for (int i = 0; i < 3; ++i) {
      int aim = cfg.accel_map[i];
      int asign = cfg.accel_sign[i];
      if (aim < 0 || aim > 2) aim = i;
      avg_accel_robot[i] = (float)asign * avg_sensor_a[aim];
    }
    madgwick.setFromAccel(avg_accel_robot[0], avg_accel_robot[1], avg_accel_robot[2]);

    // reset accumulators
    state.warm_ax_sum = state.warm_ay_sum = state.warm_az_sum = 0.0f;
    state.warm_gx_sum = state.warm_gy_sum = state.warm_gz_sum = 0.0f;
    state.warmup_samples_total = 0;
    state.gyro_bias_initialized = true;

    state.initial_persist_candidate[0] = state.gyro_bias[0];
    state.initial_persist_candidate[1] = state.gyro_bias[1];
    state.initial_persist_candidate[2] = state.gyro_bias[2];
    state.pending_initial_persist = true;
    state.initial_persist_deadline_ms = millis() + state.initial_persist_timeout_ms;
    char buf2[256];
    snprintf(buf2, sizeof(buf2), "TUNING: warmup complete (gyro_bias rad/s = %.6f, %.6f, %.6f) - persisting deferred",
             (double)state.gyro_bias[0], (double)state.gyro_bias[1], (double)state.gyro_bias[2]);
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, buf2);
  }
}

void updateBiasEmaAndPersistIfNeeded(ConsumerState &state, const IMUSample &sample, const float gyro_robot[3]) {
  if (!state.gyro_bias_initialized) return;
  float a_norm = sqrtf(sample.ax * sample.ax + sample.ay * sample.ay + sample.az * sample.az);
  const float g_nominal = 9.80665f;
  bool accel_stationary = fabsf(a_norm - g_nominal) < 0.2f;
  float ang_rate_mag = sqrtf(gyro_robot[0] * gyro_robot[0] + gyro_robot[1] * gyro_robot[1] + gyro_robot[2] * gyro_robot[2]);
  bool gyro_stationary = ang_rate_mag < 0.1f;
  if (accel_stationary && gyro_stationary) {
    float gx = sample.gx;
    float gy = sample.gy;
    float gz = sample.gz;
    state.gyro_bias[0] = (1.0f - state.bias_ema_alpha) * state.gyro_bias[0] + state.bias_ema_alpha * gx;
    state.gyro_bias[1] = (1.0f - state.bias_ema_alpha) * state.gyro_bias[1] + state.bias_ema_alpha * gy;
    state.gyro_bias[2] = (1.0f - state.bias_ema_alpha) * state.gyro_bias[2] + state.bias_ema_alpha * gz;
    uint32_t now = millis();

    if (state.pending_initial_persist) {
      if ((int32_t)(now - state.initial_persist_deadline_ms) >= 0) {
        if (fabsf(state.gyro_bias[0] - state.initial_persist_candidate[0]) <= state.persist_delta_thresh &&
            fabsf(state.gyro_bias[1] - state.initial_persist_candidate[1]) <= state.persist_delta_thresh &&
            fabsf(state.gyro_bias[2] - state.initial_persist_candidate[2]) <= state.persist_delta_thresh) {
          if (state.prefs && state.prefs_started) {
            state.prefs->putFloat("gbx", state.gyro_bias[0]);
            state.prefs->putFloat("gby", state.gyro_bias[1]);
            state.prefs->putFloat("gbz", state.gyro_bias[2]);
            state.last_persist_ms = now;
          }
          state.pending_initial_persist = false;
          LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "TUNING: initial gyro bias persisted to NVS after stability check");
        } else {
          state.initial_persist_candidate[0] = state.gyro_bias[0];
          state.initial_persist_candidate[1] = state.gyro_bias[1];
          state.initial_persist_candidate[2] = state.gyro_bias[2];
          state.initial_persist_deadline_ms = now + state.initial_persist_timeout_ms;
          LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "TUNING: initial persist deferred (candidate drift)");
        }
      }
    }

    /*if ((int32_t)(now - state.last_persist_ms) >= (int32_t)state.persist_interval_ms) {
      if (state.prefs && state.prefs_started) {
        float stored_x = state.prefs->getFloat("gbx", 0.0f);
        float stored_y = state.prefs->getFloat("gby", 0.0f);
        float stored_z = state.prefs->getFloat("gbz", 0.0f);
        if (fabsf(state.gyro_bias[0] - stored_x) > state.persist_delta_thresh ||
            fabsf(state.gyro_bias[1] - stored_y) > state.persist_delta_thresh ||
            fabsf(state.gyro_bias[2] - stored_z) > state.persist_delta_thresh) {
          state.prefs->putFloat("gbx", state.gyro_bias[0]);
          state.prefs->putFloat("gby", state.gyro_bias[1]);
          state.prefs->putFloat("gbz", state.gyro_bias[2]);
          state.last_persist_ms = now;
          LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "TUNING: persisted updated gyro bias to NVS");
        } else {
          state.last_persist_ms = now;
        }
      }
    }*/
  }
}

void publishFusedOutputsUnderMutex(ConsumerState &state,
                                   float fused_pitch_local,
                                   float fused_pitch_rate_local,
                                   float &out_fused_pitch_rad,
                                   float &out_fused_pitch_rate_rads,
                                   SemaphoreHandle_t *fusion_mutex) {
  if (fusion_mutex) {
    if (xSemaphoreTake(*fusion_mutex, (TickType_t)0) == pdTRUE) {
      out_fused_pitch_rad = fused_pitch_local;
      out_fused_pitch_rate_rads = fused_pitch_rate_local;
      xSemaphoreGive(*fusion_mutex);
    }
  }
}

void runBalancerCycleIfActive(float fused_pitch_local,
                              float fused_pitch_rate_local,
                              float dt,
                              float &left_cmd,
                              float &right_cmd) {
  if (abbot::balancer::controller::isActive() || abbot::balancer::controller::isAutotuning()) {
    (void)abbot::balancer::controller::processCycle(fused_pitch_local, fused_pitch_rate_local, dt);
    left_cmd = abbot::motor::getLastMotorCommand(LEFT_MOTOR_ID);
    right_cmd = abbot::motor::getLastMotorCommand(RIGHT_MOTOR_ID);
  }
}

void emitTuningOrStream(const ConsumerState &state,
                        const IMUSample &sample,
                        float fused_pitch_local,
                        float fused_pitch_rate_local,
                        const float accel_robot[3],
                        const float gyro_robot[3],
                        float left_cmd,
                        float right_cmd) {
  if (state.warmup_samples_remaining > 0) return;
  float pitch_deg = radToDeg(fused_pitch_local);
  float pitch_rate_dps = radToDeg(fused_pitch_rate_local);
  float pitch_rad = fused_pitch_local;
  float pitch_rate_rads = fused_pitch_rate_local;
  if (abbot::tuning::isCapturing()) {
    abbot::tuning::submitSample(sample.ts_ms,
              pitch_deg,
              pitch_rad,
              pitch_rate_dps,
              pitch_rate_rads,
              accel_robot[0], accel_robot[1], accel_robot[2],
              gyro_robot[0], gyro_robot[1], gyro_robot[2],
              sample.temp_C,
              left_cmd, right_cmd);
  } else if (abbot::log::isChannelEnabled(abbot::log::CHANNEL_TUNING)) {
    LOG_PRINTF(abbot::log::CHANNEL_TUNING,
         "%lu,%.4f,%.6f,%.4f,%.6f,%.4f,%.4f,%.4f,%.6f,%.6f,%.6f,%.3f,%.4f,%.4f\n",
         sample.ts_ms,
         pitch_deg,
         pitch_rad,
         pitch_rate_dps,
         pitch_rate_rads,
         accel_robot[0], accel_robot[1], accel_robot[2],
         gyro_robot[0], gyro_robot[1], gyro_robot[2],
         sample.temp_C,
         left_cmd,
         right_cmd);
  }
}

// --- New Helpers ---
void initializeConsumerStateFromPreferences(ConsumerState &state, Preferences &prefs) {
  state.prefs = &prefs;
  state.prefs_started = true;
  state.gyro_bias[0] = prefs.getFloat("gbx", 0.0f);
  state.gyro_bias[1] = prefs.getFloat("gby", 0.0f);
  state.gyro_bias[2] = prefs.getFloat("gbz", 0.0f);
  state.gyro_bias_initialized = true;
  state.last_persist_ms = millis();
  char buf[128];
  snprintf(buf, sizeof(buf), "TUNING: loaded persisted gyro_bias rad/s = %.6f, %.6f, %.6f",
           (double)state.gyro_bias[0], (double)state.gyro_bias[1], (double)state.gyro_bias[2]);
  LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, buf);
}

void initializeConsumerStateFromCalibrationIfNeeded(ConsumerState &state,
                                                    const abbot::imu_cal::Calibration &cal,
                                                    bool prefs_started,
                                                    Preferences *prefs) {
  const float kEPS = 1e-6f;
  bool bias_all_zero = (fabsf(state.gyro_bias[0]) < kEPS && fabsf(state.gyro_bias[1]) < kEPS && fabsf(state.gyro_bias[2]) < kEPS);
  // Acceptable delta between persisted gyro bias and IMU calibration before we decide to sync
  const float kSYNC_THRESHOLD = 1e-4f; // ~0.0057 deg/s

  if (!prefs_started || bias_all_zero) {
    // No valid persisted prefs: initialize from imu calibration
    state.gyro_bias[0] = cal.gyro_bias[0];
    state.gyro_bias[1] = cal.gyro_bias[1];
    state.gyro_bias[2] = cal.gyro_bias[2];
    state.gyro_bias_initialized = true;
    if (prefs_started && prefs) {
      prefs->putFloat("gbx", state.gyro_bias[0]);
      prefs->putFloat("gby", state.gyro_bias[1]);
      prefs->putFloat("gbz", state.gyro_bias[2]);
      state.last_persist_ms = millis();
    }
    char buf2[128];
    snprintf(buf2, sizeof(buf2), "TUNING: initialized gyro_bias from imu calibration (rad/s = %.6f, %.6f, %.6f)",
             (double)state.gyro_bias[0], (double)state.gyro_bias[1], (double)state.gyro_bias[2]);
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, buf2);
  } else if (prefs_started && prefs) {
    // We already have persisted prefs; if they differ significantly from the IMU calibration,
    // sync them so on-device calibration and persisted tuning state match.
    bool differs = false;
    for (int i = 0; i < 3; ++i) {
      float diff = fabsf(state.gyro_bias[i] - cal.gyro_bias[i]);
      if (diff > kSYNC_THRESHOLD) { differs = true; break; }
    }
    if (differs) {
      state.gyro_bias[0] = cal.gyro_bias[0];
      state.gyro_bias[1] = cal.gyro_bias[1];
      state.gyro_bias[2] = cal.gyro_bias[2];
      state.gyro_bias_initialized = true;
      prefs->putFloat("gbx", state.gyro_bias[0]);
      prefs->putFloat("gby", state.gyro_bias[1]);
      prefs->putFloat("gbz", state.gyro_bias[2]);
      state.last_persist_ms = millis();
      char buf3[128];
      snprintf(buf3, sizeof(buf3), "TUNING: synced persisted gyro_bias from imu calibration (rad/s = %.6f, %.6f, %.6f)",
               (double)state.gyro_bias[0], (double)state.gyro_bias[1], (double)state.gyro_bias[2]);
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, buf3);
    }
  }
}

void requestWarmup(ConsumerState &state, float seconds, float sample_rate_hz) {
  if (seconds <= 0.0f) return;
  float rate = sample_rate_hz > 0.0f ? sample_rate_hz : 200.0f;
  uint32_t samples = (uint32_t)ceilf(seconds * rate);
  if (samples == 0) samples = (uint32_t)rate;
  state.warmup_samples_remaining = samples;
  state.warmup_samples_total = samples;
  state.warm_ax_sum = state.warm_ay_sum = state.warm_az_sum = 0.0f;
  state.warm_gx_sum = state.warm_gy_sum = state.warm_gz_sum = 0.0f;
  char buf[128];
  snprintf(buf, sizeof(buf), "TUNING: warmup requested %u samples (~%.1f s)", (unsigned)samples, (double)seconds);
  LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, buf);
}

void emitDiagnosticsIfEnabled(uint32_t ts_ms,
                              float fused_pitch_local,
                              float fused_pitch_rate_local,
                              float left_cmd,
                              float right_cmd) {
  if (!abbot::log::isChannelEnabled(abbot::log::CHANNEL_BALANCER)) return;
  float pitch_deg = radToDeg(fused_pitch_local);
  float pitch_rate_deg = radToDeg(fused_pitch_rate_local);
  LOG_PRINTF(abbot::log::CHANNEL_BALANCER,
             "%lu,pitch_deg=%.3f,pitch_rate_deg=%.3f,left=%.3f,right=%.3f\n",
             (unsigned long)ts_ms,
             pitch_deg,
             pitch_rate_deg,
             left_cmd,
             right_cmd);
}

} // namespace imu_consumer
} // namespace abbot
