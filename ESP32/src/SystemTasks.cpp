// SystemTasks.cpp
#include "SystemTasks.h"
#include "BMI088Driver.h"
#include "imu_calibration.h"
#include "serial_commands.h"
#include "imu_fusion.h"
#include "motor_driver.h"
#include "tuning_capture.h"
#include "../config/motor_config.h"
#include "logging.h"
#include <cmath>
#include <Preferences.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <Arduino.h>

namespace abbot {

static QueueHandle_t imuQueue = nullptr;
static BMI088Driver *g_driver = nullptr;
// Optional calibration queue (single-slot). When non-null the producer will also
// write each new sample into this queue (xQueueOverwrite) so a calibration routine
// can block-read exact samples without talking to the driver directly.
static QueueHandle_t calibQueue = nullptr;

// Fusion state (shared between IMU consumer and external callers)
static unsigned long g_last_sample_timestamp_ms = 0;
static fusion::Madgwick g_madgwick;
// Store sample rate (Hz) in one place so dt fallback uses the configured value
static float g_madgwick_sample_rate_hz = 200.0f;
// Fused outputs (units: radians, radians/sec)
static float g_fused_pitch_rad = 0.0f;
static float g_fused_pitch_rate_rads = 0.0f;
static SemaphoreHandle_t g_fusion_mutex = nullptr;
// Warm-up state for tuning captures: when >0 we consume samples to let the
// Madgwick filter converge and suppress output until warmup completes.
static volatile uint32_t g_warmup_samples_remaining = 0;
static uint32_t g_warmup_samples_total = 0;
static float g_warm_ax_sum = 0.0f, g_warm_ay_sum = 0.0f, g_warm_az_sum = 0.0f;
// Also accumulate gyro samples during warm-up to estimate gyro bias
static float g_warm_gx_sum = 0.0f, g_warm_gy_sum = 0.0f, g_warm_gz_sum = 0.0f;
// Estimated gyro bias (rad/s) subtracted from gyro readings after warm-up
static float g_gyro_bias_x = 0.0f, g_gyro_bias_y = 0.0f, g_gyro_bias_z = 0.0f;
// EMA parameters for online bias tracking
static bool g_gyro_bias_initialized = false;
static const float g_bias_ema_alpha = 0.01f; // smoothing factor (small -> slow adaptation)
// Persistence (Preferences/NVS)
static Preferences g_prefs;
static bool g_prefs_started = false;
static uint32_t g_last_persist_ms = 0;
static const uint32_t g_persist_interval_ms = 60000; // write at most once per minute
static const float g_persist_delta_thresh = 0.01f; // rad/s threshold to trigger a write
// Note: tuning stream state is now managed by the logging channel manager

static void imuProducerTask(void *pvParameters) {
  BMI088Driver *driver = reinterpret_cast<BMI088Driver*>(pvParameters);
  IMUSample sample;
  for (;;) {
    // Attempt to read; BMI088Driver::read will enforce sampling interval
    if (driver->read(sample)) {
      // Single-slot overwrite keeps only the latest sample
      if (imuQueue) {
        xQueueOverwrite(imuQueue, &sample);
      }
      // If a calibration queue is attached, push sample there as well (non-blocking)
      if (calibQueue) {
        xQueueOverwrite(calibQueue, &sample);
      }
    }
    // Short delay to yield; real timing enforced by driver->read
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

static void imuConsumerTask(void *pvParameters) {
  (void)pvParameters;
  IMUSample sample;
  uint32_t last_print_ms = 0;
  // g_fusion and g_fusionMutex are initialized in startIMUTasks
  for (;;) {
    if (imuQueue && xQueueReceive(imuQueue, &sample, portMAX_DELAY) == pdTRUE) {
      // Compute dt (seconds) using host timestamp; fall back to configured sample_rate
      float dt = 1.0f / g_madgwick_sample_rate_hz;
      if (g_last_sample_timestamp_ms != 0) {
        unsigned long delta_ms = sample.ts_ms - g_last_sample_timestamp_ms;
        if (delta_ms > 0) dt = (float)delta_ms / 1000.0f;
      }
      g_last_sample_timestamp_ms = sample.ts_ms;

      // Estimate gyro bias during warm-up: accumulate raw gyro samples and
      // subtract estimated bias from readings before passing to Madgwick.
      float gx = sample.gx;
      float gy = sample.gy;
      float gz = sample.gz;

      if (g_warmup_samples_remaining > 0) {
        // accumulate accel for average orientation seed
        g_warm_ax_sum += sample.ax;
        g_warm_ay_sum += sample.ay;
        g_warm_az_sum += sample.az;
        // accumulate gyro for bias estimation
        g_warm_gx_sum += gx;
        g_warm_gy_sum += gy;
        g_warm_gz_sum += gz;
      }

      // Adjust gyro readings by subtracting the estimated bias (zero until computed)
      float gx_adj = gx - g_gyro_bias_x;
      float gy_adj = gy - g_gyro_bias_y;
      float gz_adj = gz - g_gyro_bias_z;

      // Update Madgwick filter with bias-compensated gyro and accel
      g_madgwick.update(gx_adj, gy_adj, gz_adj, sample.ax, sample.ay, sample.az, dt);

      // If warm-up is requested, decrement counter and finalize when done
      if (g_warmup_samples_remaining > 0) {
        // decrement remaining
        --g_warmup_samples_remaining;
        // If warmup just completed, compute averages and seed Madgwick
        if (g_warmup_samples_remaining == 0) {
          float ax_avg = g_warm_ax_sum / (float)g_warmup_samples_total;
          float ay_avg = g_warm_ay_sum / (float)g_warmup_samples_total;
          float az_avg = g_warm_az_sum / (float)g_warmup_samples_total;
          // compute gyro bias (rad/s)
          g_gyro_bias_x = g_warm_gx_sum / (float)g_warmup_samples_total;
          g_gyro_bias_y = g_warm_gy_sum / (float)g_warmup_samples_total;
          g_gyro_bias_z = g_warm_gz_sum / (float)g_warmup_samples_total;
          // seed orientation from accel average
          g_madgwick.setFromAccel(ax_avg, ay_avg, az_avg);
          // reset accumulators
          g_warm_ax_sum = g_warm_ay_sum = g_warm_az_sum = 0.0f;
          g_warm_gx_sum = g_warm_gy_sum = g_warm_gz_sum = 0.0f;
          g_warmup_samples_total = 0;
          // mark bias as initialized so EMA updates can run
          g_gyro_bias_initialized = true;
          // persist initial bias to NVS immediately (but avoid hammering flash)
          uint32_t now_ms = millis();
          if (g_prefs_started) {
            g_prefs.putFloat("gbx", g_gyro_bias_x);
            g_prefs.putFloat("gby", g_gyro_bias_y);
            g_prefs.putFloat("gbz", g_gyro_bias_z);
            g_last_persist_ms = now_ms;
          }
          char buf2[128];
          snprintf(buf2, sizeof(buf2), "TUNING: warmup complete (gyro_bias rad/s = %.6f, %.6f, %.6f)",
                   (double)g_gyro_bias_x, (double)g_gyro_bias_y, (double)g_gyro_bias_z);
          LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, buf2);
        }
      }

      // After warmup we can attempt ongoing online bias refinement when robot is stationary.
      // Stationary detection: accel magnitude near 1g and low angular rate.
      if (g_gyro_bias_initialized) {
        // accel magnitude (m/s^2)
        float a_norm = sqrtf(sample.ax * sample.ax + sample.ay * sample.ay + sample.az * sample.az);
        const float g_nominal = 9.80665f;
        bool accel_stationary = fabsf(a_norm - g_nominal) < 0.2f; // ~0.02g tolerance
        // low angular rate (use the bias-compensated rates)
        float ang_rate_mag = sqrtf(gx_adj * gx_adj + gy_adj * gy_adj + gz_adj * gz_adj);
        bool gyro_stationary = ang_rate_mag < 0.1f; // ~0.1 rad/s (~5.7 deg/s)
        if (accel_stationary && gyro_stationary) {
          // Use raw gyro sample (gx,gy,gz) as an instantaneous bias measurement (true motion small)
          // Update EMA toward that measured value
          g_gyro_bias_x = (1.0f - g_bias_ema_alpha) * g_gyro_bias_x + g_bias_ema_alpha * gx;
          g_gyro_bias_y = (1.0f - g_bias_ema_alpha) * g_gyro_bias_y + g_bias_ema_alpha * gy;
          g_gyro_bias_z = (1.0f - g_bias_ema_alpha) * g_gyro_bias_z + g_bias_ema_alpha * gz;
          // Persist occasionally if bias changed enough and interval passed
          uint32_t now = millis();
          if ((int32_t)(now - g_last_persist_ms) >= (int32_t)g_persist_interval_ms) {
            // read current stored values and decide if writing is needed
            float stored_x = g_prefs.getFloat("gbx", 0.0f);
            float stored_y = g_prefs.getFloat("gby", 0.0f);
            float stored_z = g_prefs.getFloat("gbz", 0.0f);
            if (fabsf(g_gyro_bias_x - stored_x) > g_persist_delta_thresh ||
                fabsf(g_gyro_bias_y - stored_y) > g_persist_delta_thresh ||
                fabsf(g_gyro_bias_z - stored_z) > g_persist_delta_thresh) {
              g_prefs.putFloat("gbx", g_gyro_bias_x);
              g_prefs.putFloat("gby", g_gyro_bias_y);
              g_prefs.putFloat("gbz", g_gyro_bias_z);
              g_last_persist_ms = now;
              LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "TUNING: persisted updated gyro bias to NVS");
            } else {
              // still update timestamp to avoid repeated reads if nothing changed
              g_last_persist_ms = now;
            }
          }
        }
      }

      // Read fused outputs and store under mutex
      float fused_pitch_local = g_madgwick.getPitch();
      float fused_pitch_rate_local = g_madgwick.getPitchRate();
      if (g_fusion_mutex) {
        if (xSemaphoreTake(g_fusion_mutex, (TickType_t)0) == pdTRUE) {
          g_fused_pitch_rad = fused_pitch_local;
          g_fused_pitch_rate_rads = fused_pitch_rate_local;
          xSemaphoreGive(g_fusion_mutex);
        }
      }

      // Tuning output: two modes exist
      // - Stream mode: `startTuningStream()` enables CHANNEL_TUNING and SystemTasks should print CSV lines
      // - Capture mode: `startCapture()` delegates CSV emission and stats to the tuning helper
      const float RAD2DEG = 57.29577951308232f; // 180/pi
      float pitch_deg = fused_pitch_local * RAD2DEG;
      float pitch_rate_dps = fused_pitch_rate_local * RAD2DEG;
      float pitch_rad = fused_pitch_local;
      float pitch_rate_rads = fused_pitch_rate_local;
      // Read last motor commands (normalized) via motor driver API
      float left_cmd = abbot::motor::getLastMotorCommand(LEFT_MOTOR_ID);
      float right_cmd = abbot::motor::getLastMotorCommand(RIGHT_MOTOR_ID);

      if (g_warmup_samples_remaining > 0) {
        // Suppress emission while warmup in progress
        continue;
      }

      if (abbot::tuning::isCapturing()) {
        // In capture mode the helper is responsible for CSV emission and statistics
        abbot::tuning::submitSample(sample.ts_ms,
                                    pitch_deg,
                                    pitch_rad,
                                    pitch_rate_dps,
                                    pitch_rate_rads,
                                    sample.ax, sample.ay, sample.az,
                                    sample.gx, sample.gy, sample.gz,
                                    // BMI088 temp from accel
                                    sample.temp_C,
                                    left_cmd, right_cmd);
        
      } else if (abbot::log::isChannelEnabled(abbot::log::CHANNEL_TUNING)) {
        // Stream mode: print CSV here
        LOG_PRINTF(abbot::log::CHANNEL_TUNING,
                   "%lu,%.4f,%.6f,%.4f,%.6f,%.4f,%.4f,%.4f,%.6f,%.6f,%.6f,%.3f,%.4f,%.4f\n",
                   sample.ts_ms,
                   pitch_deg,
                   pitch_rad,
                   pitch_rate_dps,
                   pitch_rate_rads,
                   sample.ax, sample.ay, sample.az,
                   sample.gx, sample.gy, sample.gz,
                   sample.temp_C,
                   left_cmd,
                   right_cmd);
      }
      #if defined(ENABLE_DEBUG_LOGS)
      // Suppress debug logs while calibration runs
      if (abbot::imu_cal::isCalibrating()) {
        continue;
      }
      // Throttle logging to once every 1000 ms to avoid flooding the serial
      uint32_t now = millis();
      if ((uint32_t)(now - last_print_ms) >= 1000u) {
        last_print_ms = now;
        LOG_PRINTF(abbot::log::CHANNEL_IMU,
             "IMU ts_ms=%lu ax=%.6f ay=%.6f az=%.6f gx=%.6f gy=%.6f gz=%.6f\n",
             sample.ts_ms,
             sample.ax, sample.ay, sample.az,
             sample.gx, sample.gy, sample.gz);
      }
      #endif
    }
  }
}

bool startIMUTasks(BMI088Driver *driver) {
  if (!driver) {
    return false;
  }
  g_driver = driver;
  if (!imuQueue) {
    imuQueue = xQueueCreate(1, sizeof(IMUSample));
    if (!imuQueue) {
      return false;
    }
  }

  // Initialize fusion mutex and default config
  if (!g_fusion_mutex) {
    g_fusion_mutex = xSemaphoreCreateMutex();
    fusion::FusionConfig cfg;
    cfg.beta = 0.1f;
    // Use the BMI088 driver's configured sampling rate if available
    if (driver) {
      cfg.sample_rate = (float)driver->getSamplingHz();
    } else {
      cfg.sample_rate = 200.0f;
    }
    g_madgwick_sample_rate_hz = cfg.sample_rate;
    g_madgwick.begin(cfg);
  }

  // Initialize logging manager and ensure TUNING channel is disabled by default
  abbot::log::init();
  abbot::log::disableChannel(abbot::log::CHANNEL_TUNING);

  // Initialize Preferences (NVS) and try to load persisted gyro bias
  // Use namespace "abbot" in Preferences. If NVS isn't available this is a no-op.
  if (g_prefs.begin("abbot", false)) {
    g_prefs_started = true;
    g_gyro_bias_x = g_prefs.getFloat("gbx", 0.0f);
    g_gyro_bias_y = g_prefs.getFloat("gby", 0.0f);
    g_gyro_bias_z = g_prefs.getFloat("gbz", 0.0f);
    g_gyro_bias_initialized = true;
    g_last_persist_ms = millis();
    char buf[128];
    snprintf(buf, sizeof(buf), "TUNING: loaded persisted gyro_bias rad/s = %.6f, %.6f, %.6f",
             (double)g_gyro_bias_x, (double)g_gyro_bias_y, (double)g_gyro_bias_z);
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, buf);
  }

  // If no persisted `abbot` gyro bias was found (or Preferences unavailable),
  // try to initialize bias from the IMU calibration stored under the "imu"
  // namespace. This synchronizes the calibration that `imu_cal` manages with
  // the runtime bias used by the Madgwick filter.
  abbot::imu_cal::Calibration cal;
  bool have_imu_cal = abbot::imu_cal::loadCalibration(cal);
  if (have_imu_cal) {
    // Check if we should prefer imu_cal values: if Preferences didn't start
    // or if persisted abbot bias is effectively zero, populate from imu cal.
    const float kEPS = 1e-6f;
    bool abbot_bias_all_zero = (fabsf(g_gyro_bias_x) < kEPS && fabsf(g_gyro_bias_y) < kEPS && fabsf(g_gyro_bias_z) < kEPS);
    if (!g_prefs_started || abbot_bias_all_zero) {
      g_gyro_bias_x = cal.gyro_bias[0];
      g_gyro_bias_y = cal.gyro_bias[1];
      g_gyro_bias_z = cal.gyro_bias[2];
      g_gyro_bias_initialized = true;
      // Persist into abbot namespace if available
      if (g_prefs_started) {
        g_prefs.putFloat("gbx", g_gyro_bias_x);
        g_prefs.putFloat("gby", g_gyro_bias_y);
        g_prefs.putFloat("gbz", g_gyro_bias_z);
        g_last_persist_ms = millis();
      }
      char buf2[128];
      snprintf(buf2, sizeof(buf2), "TUNING: initialized gyro_bias from imu calibration (rad/s = %.6f, %.6f, %.6f)",
               (double)g_gyro_bias_x, (double)g_gyro_bias_y, (double)g_gyro_bias_z);
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, buf2);
    }
  }

  // Create producer task (higher priority)
  BaseType_t r1 = xTaskCreate(imuProducerTask, "IMUProducer", 4096, driver, configMAX_PRIORITIES - 2, nullptr);
  // Create consumer task (lower priority)
  BaseType_t r2 = xTaskCreate(imuConsumerTask, "IMUConsumer", 4096, nullptr, configMAX_PRIORITIES - 3, nullptr);

  // Create serial command task for calibration UI (low priority)
  BaseType_t r3 = xTaskCreate(abbot::serialcmds::serialTaskEntry, "IMUSerial", 4096, driver, configMAX_PRIORITIES - 4, nullptr);

  return (r1 == pdPASS) && (r2 == pdPASS) && (r3 == pdPASS);
}

void attachCalibrationQueue(QueueHandle_t q) {
  calibQueue = q;
}

void detachCalibrationQueue() {
  calibQueue = nullptr;
}

float getFusedPitch() {
  float out = 0.0f;
  if (g_fusion_mutex) {
    if (xSemaphoreTake(g_fusion_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      out = g_fused_pitch_rad;
      xSemaphoreGive(g_fusion_mutex);
      return out;
    }
  }
  return out;
}

float getFusedPitchRate() {
  float out = 0.0f;
  if (g_fusion_mutex) {
    if (xSemaphoreTake(g_fusion_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      out = g_fused_pitch_rate_rads;
      xSemaphoreGive(g_fusion_mutex);
      return out;
    }
  }
  return out;
}

void startTuningStream() {
  // enable logging channel and print CSV header
  abbot::log::enableChannel(abbot::log::CHANNEL_TUNING);
  // Print CSV header for consumers (include raw accel/gyro and temp)
  LOG_PRINTLN(abbot::log::CHANNEL_TUNING, "timestamp_ms,pitch_deg,pitch_rad,pitch_rate_deg,pitch_rate_rad,ax,ay,az,gx,gy,gz,temp_C,left_cmd,right_cmd");
}

void stopTuningStream() {
  abbot::log::disableChannel(abbot::log::CHANNEL_TUNING);
}

bool isTuningStreamActive() {
  return abbot::log::isChannelEnabled(abbot::log::CHANNEL_TUNING);
}

void requestTuningWarmupSeconds(float seconds) {
  if (seconds <= 0.0f) {
    return;
  }
  // compute samples based on configured madgwick sample rate (fallback if 0)
  float rate = g_madgwick_sample_rate_hz;
  if (rate <= 0.0f) {
    rate = 200.0f;
  }
  uint32_t samples = (uint32_t)ceilf(seconds * rate);
  if (samples == 0) {
    samples = (uint32_t)rate; // at least one second
  }
  g_warmup_samples_remaining = samples;
  g_warmup_samples_total = samples;
  g_warm_ax_sum = g_warm_ay_sum = g_warm_az_sum = 0.0f;
  char buf[128];
  snprintf(buf, sizeof(buf), "TUNING: warmup requested %u samples (~%.1f s)", (unsigned)samples, (double)seconds);
  LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, buf);
}

} // namespace abbot
