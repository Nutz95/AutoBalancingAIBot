#include "imu_consumer_helpers.h"
#include "SystemTasks.h" // For ENABLE_DEBUG_LOGS
#include "../config/imu_filter_config.h"
#include "../config/balancer_config.h"
#include "../include/balancer_controller.h"
#include "imu_calibration.h"
#include "imu_mapping.h"
#include "logging.h"
#include "motor_drivers/driver_manager.h"
#include "tuning_capture.h"
#include "units.h"
#include "TelemetryService.h"
#include <cmath>
#include <cstdio>

namespace abbot {
namespace serialcmds {
extern bool tryGetCpuLoadPercent(float &cpu0, float &cpu1);
}

namespace imu_consumer {

float computeDt(ConsumerState &state, const IMUSample &sample,
                float sample_rate_hz) {
  float dt = 1.0f / sample_rate_hz;
  if (state.last_sample_timestamp_us != 0) {
    unsigned long delta_us = sample.ts_us - state.last_sample_timestamp_us;
    if (delta_us > 0)
      dt = (float)delta_us / 1000000.0f;
  }
  state.last_sample_timestamp_us = sample.ts_us;
  return dt;
}

void accumulateWarmup(ConsumerState &state, const IMUSample &sample, float gx,
                      float gy, float gz) {
  if (state.warmup_samples_remaining > 0) {
    state.warm_ax_sum += sample.ax;
    state.warm_ay_sum += sample.ay;
    state.warm_az_sum += sample.az;
    state.warm_gx_sum += gx;
    state.warm_gy_sum += gy;
    state.warm_gz_sum += gz;
  }
}

void finalizeWarmupIfDone(ConsumerState &state, abbot::IMUFilter &filter,
                          const fusion::FusionConfig &cfg) {
  if (state.warmup_samples_remaining == 0 && state.warmup_samples_total > 0) {
    float ax_avg = state.warm_ax_sum / (float)state.warmup_samples_total;
    float ay_avg = state.warm_ay_sum / (float)state.warmup_samples_total;
    float az_avg = state.warm_az_sum / (float)state.warmup_samples_total;
    state.gyro_bias[0] = state.warm_gx_sum / (float)state.warmup_samples_total;
    state.gyro_bias[1] = state.warm_gy_sum / (float)state.warmup_samples_total;
    state.gyro_bias[2] = state.warm_gz_sum / (float)state.warmup_samples_total;

    float avg_sensor_a[3] = {ax_avg, ay_avg, az_avg};
    float avg_accel_robot[3];
    for (int i = 0; i < 3; ++i) {
      int aim = cfg.accel_map[i];
      int asign = cfg.accel_sign[i];
      if (aim < 0 || aim > 2)
        aim = i;
      avg_accel_robot[i] = (float)asign * avg_sensor_a[aim];
    }
    // Let the filter seed itself from accel averages if it implements it
    filter.setFromAccel(avg_accel_robot[0], avg_accel_robot[1],
                        avg_accel_robot[2]);

    // reset accumulators
    state.warm_ax_sum = state.warm_ay_sum = state.warm_az_sum = 0.0f;
    state.warm_gx_sum = state.warm_gy_sum = state.warm_gz_sum = 0.0f;
    state.warmup_samples_total = 0;
    state.gyro_bias_initialized = true;

    state.initial_persist_candidate[0] = state.gyro_bias[0];
    state.initial_persist_candidate[1] = state.gyro_bias[1];
    state.initial_persist_candidate[2] = state.gyro_bias[2];
    state.pending_initial_persist = true;
    state.initial_persist_deadline_ms =
        millis() + state.initial_persist_timeout_ms;
    char buf2[256];
    snprintf(buf2, sizeof(buf2),
             "TUNING: warmup complete (gyro_bias rad/s = %.6f, %.6f, %.6f) - "
             "persisting deferred",
             (double)state.gyro_bias[0], (double)state.gyro_bias[1],
             (double)state.gyro_bias[2]);
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, buf2);
  }
}

/**
 * @brief Persists the current gyro bias to NVS if it has changed significantly.
 * 
 * This is called periodically when the robot is stationary to ensure the 
 * learned bias survives a reboot.
 */
void persistBiasToNVS(ConsumerState &state, uint32_t now) {
  if (!state.prefs || !state.prefs_started) {
    return;
  }

  float stored_x = state.prefs->getFloat("gbx", 0.0f);
  float stored_y = state.prefs->getFloat("gby", 0.0f);
  float stored_z = state.prefs->getFloat("gbz", 0.0f);

  bool changed = fabsf(state.gyro_bias[0] - stored_x) > state.persist_delta_thresh ||
                 fabsf(state.gyro_bias[1] - stored_y) > state.persist_delta_thresh ||
                 fabsf(state.gyro_bias[2] - stored_z) > state.persist_delta_thresh;

  if (changed) {
    state.prefs->putFloat("gbx", state.gyro_bias[0]);
    state.prefs->putFloat("gby", state.gyro_bias[1]);
    state.prefs->putFloat("gbz", state.gyro_bias[2]);
    state.last_persist_ms = now;
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "TUNING: persisted updated gyro bias to NVS");
  } else {
    state.last_persist_ms = now;
  }
}

void updateBiasEmaAndPersistIfNeeded(ConsumerState &state,
                                     const IMUSample &sample,
                                     const float gyro_robot[3]) {
#if IMU_AUTO_GYRO_BIAS_UPDATE
  if (!state.gyro_bias_initialized) {
    return;
  }

  // Check if robot is stationary (both accel and gyro)
  float a_norm = sqrtf(sample.ax * sample.ax + sample.ay * sample.ay +
                       sample.az * sample.az);
  const float g_nominal = 9.80665f;
  bool accel_stationary = fabsf(a_norm - g_nominal) < 0.2f;

  // We use the bias-corrected robot-frame gyro to detect motion.
  // This checks if the current rate is close to our current bias estimate.
  float ang_rate_mag =
      sqrtf(gyro_robot[0] * gyro_robot[0] + gyro_robot[1] * gyro_robot[1] +
            gyro_robot[2] * gyro_robot[2]);
  bool gyro_stationary = ang_rate_mag < IMU_GYRO_STATIONARY_THRESHOLD_RAD_S;

  if (accel_stationary && gyro_stationary) {
    // Update bias EMA using raw sensor-frame values.
    // The bias is the average raw value when the sensor is stationary.
    state.gyro_bias[0] = (1.0f - state.bias_ema_alpha) * state.gyro_bias[0] +
                         state.bias_ema_alpha * sample.gx;
    state.gyro_bias[1] = (1.0f - state.bias_ema_alpha) * state.gyro_bias[1] +
                         state.bias_ema_alpha * sample.gy;
    state.gyro_bias[2] = (1.0f - state.bias_ema_alpha) * state.gyro_bias[2] +
                         state.bias_ema_alpha * sample.gz;

    uint32_t now = millis();

    // Handle initial persistence after warmup stability check
    if (state.pending_initial_persist) {
      if ((int32_t)(now - state.initial_persist_deadline_ms) >= 0) {
        bool stable = fabsf(state.gyro_bias[0] - state.initial_persist_candidate[0]) <= state.persist_delta_thresh &&
                      fabsf(state.gyro_bias[1] - state.initial_persist_candidate[1]) <= state.persist_delta_thresh &&
                      fabsf(state.gyro_bias[2] - state.initial_persist_candidate[2]) <= state.persist_delta_thresh;

        if (stable) {
          persistBiasToNVS(state, now);
          state.pending_initial_persist = false;
          LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT,
                      "TUNING: initial gyro bias persisted to NVS after stability check");
        } else {
          state.initial_persist_candidate[0] = state.gyro_bias[0];
          state.initial_persist_candidate[1] = state.gyro_bias[1];
          state.initial_persist_candidate[2] = state.gyro_bias[2];
          state.initial_persist_deadline_ms = now + state.initial_persist_timeout_ms;
          LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "TUNING: initial persist deferred (candidate drift)");
        }
      }
    }

    // Periodic persistence
    if ((int32_t)(now - state.last_persist_ms) >= (int32_t)state.persist_interval_ms) {
      persistBiasToNVS(state, now);
    }
  }
#endif
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
                              float fused_pitch_rate_local, float dt,
                              const float accel_robot[3],
                              const float gyro_robot[3],
                              float &left_cmd, float &right_cmd) {
  if (abbot::balancer::controller::isActive() ||
      abbot::balancer::controller::isAutotuning()) {
    abbot::balancer::controller::setLatestImuSample(accel_robot, gyro_robot);
    (void)abbot::balancer::controller::processCycle(fused_pitch_local,
                                                    fused_pitch_rate_local, dt);
    if (auto drv = abbot::motor::getActiveMotorDriver()) {
      // Query last commands by MotorSide to avoid depending on driver-specific
      // IDs
      left_cmd =
          drv->getLastMotorCommand(abbot::motor::IMotorDriver::MotorSide::LEFT);
      right_cmd = drv->getLastMotorCommand(
          abbot::motor::IMotorDriver::MotorSide::RIGHT);
    } else {
      left_cmd = 0.0f;
      right_cmd = 0.0f;
    }
  }
}

void emitTuningOrStream(const ConsumerState &state, const IMUSample &sample,
                        float fused_pitch_local, float fused_pitch_rate_local,
                        const float accel_robot[3], const float gyro_robot[3],
                        float left_cmd, float right_cmd) {
  if (state.warmup_samples_remaining > 0)
    return;
  float pitch_deg = radToDeg(fused_pitch_local);
  float pitch_rate_dps = radToDeg(fused_pitch_rate_local);
  float pitch_rad = fused_pitch_local;
  float pitch_rate_rads = fused_pitch_rate_local;
  if (abbot::tuning::isCapturing()) {
    abbot::tuning::submitSample(
        sample.ts_ms, pitch_deg, pitch_rad, pitch_rate_dps, pitch_rate_rads,
        accel_robot[0], accel_robot[1], accel_robot[2], gyro_robot[0],
        gyro_robot[1], gyro_robot[2], sample.temperatureCelsius, left_cmd,
        right_cmd);
  } else if (abbot::log::isChannelEnabled(abbot::log::CHANNEL_TUNING)) {
    LOG_PRINTF(abbot::log::CHANNEL_TUNING,
               "%lu,%.4f,%.6f,%.4f,%.6f,%.4f,%.4f,%.4f,%.6f,%.6f,%.6f,%.3f,%."
               "4f,%.4f\n",
               sample.ts_ms, pitch_deg, pitch_rad, pitch_rate_dps,
               pitch_rate_rads, accel_robot[0], accel_robot[1], accel_robot[2],
               gyro_robot[0], gyro_robot[1], gyro_robot[2],
               sample.temperatureCelsius, left_cmd, right_cmd);
  }
}

// --- New Helpers ---
void initializeConsumerStateFromPreferences(ConsumerState &state,
                                            Preferences &prefs) {
  state.prefs = &prefs;
  state.prefs_started = true;
  state.gyro_bias[0] = prefs.getFloat("gbx", 0.0f);
  state.gyro_bias[1] = prefs.getFloat("gby", 0.0f);
  state.gyro_bias[2] = prefs.getFloat("gbz", 0.0f);
  state.gyro_bias_initialized = true;
  state.last_persist_ms = millis();
  char buf[128];
  snprintf(buf, sizeof(buf),
           "TUNING: loaded persisted gyro_bias rad/s = %.6f, %.6f, %.6f",
           (double)state.gyro_bias[0], (double)state.gyro_bias[1],
           (double)state.gyro_bias[2]);
  LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, buf);
}

void initializeConsumerStateFromCalibrationIfNeeded(
    ConsumerState &state, const abbot::imu_cal::Calibration &cal,
    bool prefs_started, Preferences *prefs) {
  const float kEPS = 1e-6f;
  bool bias_all_zero =
      (fabsf(state.gyro_bias[0]) < kEPS && fabsf(state.gyro_bias[1]) < kEPS &&
       fabsf(state.gyro_bias[2]) < kEPS);
  // Acceptable delta between persisted gyro bias and IMU calibration before we
  // decide to sync
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
    snprintf(buf2, sizeof(buf2),
             "TUNING: initialized gyro_bias from imu calibration (rad/s = "
             "%.6f, %.6f, %.6f)",
             (double)state.gyro_bias[0], (double)state.gyro_bias[1],
             (double)state.gyro_bias[2]);
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, buf2);
  } else if (prefs_started && prefs) {
    // We already have persisted prefs; if they differ significantly from the
    // IMU calibration, sync them so on-device calibration and persisted tuning
    // state match.
    bool differs = false;
    for (int i = 0; i < 3; ++i) {
      float diff = fabsf(state.gyro_bias[i] - cal.gyro_bias[i]);
      if (diff > kSYNC_THRESHOLD) {
        differs = true;
        break;
      }
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
      snprintf(buf3, sizeof(buf3),
               "TUNING: synced persisted gyro_bias from imu calibration (rad/s "
               "= %.6f, %.6f, %.6f)",
               (double)state.gyro_bias[0], (double)state.gyro_bias[1],
               (double)state.gyro_bias[2]);
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, buf3);
    }
  }
}

void requestWarmup(ConsumerState &state, float seconds, float sample_rate_hz) {
  if (seconds <= 0.0f)
    return;
  float rate = sample_rate_hz > 0.0f ? sample_rate_hz : 200.0f;
  uint32_t samples = (uint32_t)ceilf(seconds * rate);
  if (samples == 0)
    samples = (uint32_t)rate;
  state.warmup_samples_remaining = samples;
  state.warmup_samples_total = samples;
  state.warm_ax_sum = state.warm_ay_sum = state.warm_az_sum = 0.0f;
  state.warm_gx_sum = state.warm_gy_sum = state.warm_gz_sum = 0.0f;
  char buf[128];
  snprintf(buf, sizeof(buf), "TUNING: warmup requested %u samples (~%.1f s)",
           (unsigned)samples, (double)seconds);
  LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, buf);
}

void emitDiagnosticsIfEnabled(uint32_t ts_ms, float fused_pitch_local,
                              float fused_pitch_rate_local, float left_cmd,
                              float right_cmd, const float accel_robot[3], 
                              const float gyro_robot[3], float freq_hz, uint32_t lat_us,
                              uint32_t ack_pending_left_us,
                              uint32_t ack_pending_right_us,
                              const ProfileData &profiler) {
  static uint32_t last_log_ms = 0;
  static uint8_t telemetry_divider = 0;
  uint32_t now = millis();
  
  float pitch_deg = radToDeg(fused_pitch_local);
  float avg_cmd = (left_cmd + right_cmd) / 2.0f;
  float steer = (left_cmd - right_cmd) / 2.0f;

  // Binary UDP Telemetry - Throttled to reduce CPU load
  // This helps core 0 handle WiFi and Bluetooth (NimBLE) reliably.
  if (abbot::telemetry::TelemetryService::getInstance().isActive()) {
      if (++telemetry_divider >= TELEMETRY_BINARY_DIVIDER) {
          telemetry_divider = 0;
          
          abbot::balancer::controller::Diagnostics diag = {};
          abbot::balancer::controller::getDiagnostics(diag);

          abbot::telemetry::TelemetryPacket pkt;
          pkt.timestamp_ms = ts_ms;
      pkt.pitch_deg = pitch_deg;
      pkt.pid_in_deg = pitch_deg;
      pkt.pid_out = avg_cmd;
      pkt.iterm = diag.iterm;
      pkt.cmd = avg_cmd;
      pkt.steer = steer;
      pkt.ax = accel_robot[0]; pkt.ay = accel_robot[1]; pkt.az = accel_robot[2];
      pkt.gx = gyro_robot[0]; pkt.gy = gyro_robot[1]; pkt.gz = gyro_robot[2];
      pkt.loop_freq_hz = freq_hz;
      pkt.enc_l = diag.enc_l; pkt.enc_r = diag.enc_r;
      pkt.bus_latency_us = lat_us;
      pkt.ack_pending_left_us = ack_pending_left_us;
      pkt.ack_pending_right_us = ack_pending_right_us;
      pkt.lqr_angle = diag.lqr_angle;
      pkt.lqr_gyro = diag.lqr_gyro;
      pkt.lqr_dist = diag.lqr_dist;
      pkt.lqr_speed = diag.lqr_speed;
      
      static uint32_t last_cpu_ms = 0;
      static float last_c0 = 0, last_c1 = 0;
      if (now - last_cpu_ms >= 100) {
          if (abbot::serialcmds::tryGetCpuLoadPercent(last_c0, last_c1)) {
              last_cpu_ms = now;
          }
      }
      pkt.cpu0_pct = last_c0;
      pkt.cpu1_pct = last_c1;

      pkt.prof_f = profiler.t_fusion;
      pkt.prof_l = profiler.t_lqr;
      pkt.prof_t = profiler.t_total;
      pkt.prof_log = profiler.t_logging;
      abbot::telemetry::TelemetryService::getInstance().send(pkt);
      }
  }

  if (!abbot::log::isChannelEnabled(abbot::log::CHANNEL_BALANCER)) {
    return;
  }

  // Throttle to avoid serial saturation which causes loop frequency drops
  if (now - last_log_ms < BALANCER_DEBUG_LOG_INTERVAL_MS) {
    return;
  }

  // Fallback to text logs if enabled, but use a higher throttle if binary is active
  if (abbot::telemetry::TelemetryService::getInstance().isActive()) {
      if (now - last_log_ms < TELEMETRY_TEXT_THROTTLE_MS) {
          return;
      }
  }
  last_log_ms = now;

  abbot::balancer::controller::Diagnostics diag = {};
  abbot::balancer::controller::getDiagnostics(diag);

  LOG_PRINTF(abbot::log::CHANNEL_BALANCER,
             "BALANCER_DBG t=%lums pitch=%.3fdeg pid_in=%.3fdeg pid_out=%.3f iterm=%.3f cmd=%.3f steer=%.3f lat=%luus ax=%.3f ay=%.3f az=%.3f gx=%.3f gy=%.3f gz=%.3f lp_hz=%.1f encL=%ld encR=%ld termA=%.6f termG=%.6f termD=%.6f termS=%.6f prof_f=%lu prof_l=%lu prof_t=%lu\n",
             (unsigned long)ts_ms, pitch_deg, pitch_deg, avg_cmd, diag.iterm, avg_cmd, steer,
             (unsigned long)lat_us, accel_robot[0], accel_robot[1], accel_robot[2], 
             gyro_robot[0], gyro_robot[1], gyro_robot[2], freq_hz, (long)diag.enc_l, (long)diag.enc_r,
             diag.lqr_angle, diag.lqr_gyro, diag.lqr_dist, diag.lqr_speed,
             (unsigned long)profiler.t_fusion, (unsigned long)profiler.t_lqr, (unsigned long)profiler.t_total);
}

bool measureAndLogImuFrequency(ImuFrequencyMeasurement &freq_state,
                               float target_hz) {
  uint32_t now_ms = millis();
  
  // Initialize start time on first call
  if (freq_state.start_ms == 0) {
    freq_state.start_ms = now_ms;
  }
  
  freq_state.sample_count++;
  
  uint32_t elapsed_ms = now_ms - freq_state.start_ms;
  if (elapsed_ms >= ImuFrequencyMeasurement::kLogIntervalMs) {
    float measured_hz = (float)freq_state.sample_count / ((float)elapsed_ms / 1000.0f);
    char freq_msg[128];
    snprintf(freq_msg, sizeof(freq_msg),
             "IMU: freq measured=%.1f Hz target=%.1f Hz (%.1f%%)",
             (double)measured_hz, (double)target_hz,
             (double)(100.0f * measured_hz / target_hz));
    LOG_PRINTLN(abbot::log::CHANNEL_IMU, freq_msg);
    
    // Reset counters
    freq_state.sample_count = 0;
    freq_state.start_ms = now_ms;
    return true;
  }
  return false;
}

void mapSensorToRobotFrame(const fusion::FusionConfig &cfg,
                           const IMUSample &sample,
                           const float gyro_bias[3],
                           float gyro_robot[3],
                           float accel_robot[3]) {
  float raw_g[3] = {sample.gx, sample.gy, sample.gz};
  float raw_a[3] = {sample.ax, sample.ay, sample.az};
  abbot::imu_mapping::mapSensorToRobot(cfg, raw_g, raw_a, gyro_bias,
                                       gyro_robot, accel_robot);
}

void getLastMotorCommands(float &left_cmd, float &right_cmd) {
  left_cmd = 0.0f;
  right_cmd = 0.0f;
  if (auto drv = abbot::motor::getActiveMotorDriver()) {
    left_cmd = drv->getLastMotorCommand(
        abbot::motor::IMotorDriver::MotorSide::LEFT);
    right_cmd = drv->getLastMotorCommand(
        abbot::motor::IMotorDriver::MotorSide::RIGHT);
  }
}

bool receiveLatestSample(QueueHandle_t queue, IMUSample &sample, uint32_t wait_ticks) {
  if (!queue) {
    return false;
  }

  // Wait for at least one sample
  if (xQueueReceive(queue, &sample, wait_ticks) != pdTRUE) {
    return false;
  }

  // Drain the rest of the queue to get the freshest sample
  IMUSample next_sample;
  while (xQueueReceive(queue, &next_sample, 0) == pdTRUE) {
    sample = next_sample;
  }

  return true;
}

void emitImuDebugLogsIfEnabled(const IMUSample &sample, uint32_t &last_print_ms,
                               uint32_t interval_ms) {
#if defined(ENABLE_DEBUG_LOGS)
  // Suppress debug logs while calibration runs
  if (abbot::imu_cal::isCalibrating()) {
    return;
  }
  uint32_t now = millis();
  if ((uint32_t)(now - last_print_ms) >= interval_ms) {
    last_print_ms = now;
    LOG_PRINTF(
        abbot::log::CHANNEL_IMU,
        "IMU ts_ms=%lu ax=%.4f ay=%.4f az=%.4f gx=%.4f gy=%.4f gz=%.4f fp=%.2f fr=%.2f fy=%.2f\n",
        sample.ts_ms, sample.ax, sample.ay, sample.az, sample.gx, sample.gy,
        sample.gz, sample.fused_pitch * 57.2957795f, sample.fused_roll * 57.2957795f,
        sample.fused_yaw * 57.2957795f);
  }
#else
  (void)sample;
  (void)last_print_ms;
  (void)interval_ms;
#endif
}

} // namespace imu_consumer
} // namespace abbot
