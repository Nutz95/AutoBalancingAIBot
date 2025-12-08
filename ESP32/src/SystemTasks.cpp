#include "units.h"
// SystemTasks.cpp
#include "SystemTasks.h"
#include "BMI088Driver.h"
#include "imu_calibration.h"
#include "serial_commands.h"
#include "imu_fusion.h"
#include "imu_mapping.h"
#include "imu_consumer_helpers.h"
#include "imu_filter.h"
#include "../config/imu_filter_config.h"
#include "motor_driver.h"
#include "tuning_capture.h"
#include "../include/balancer_controller.h"
#include "../config/motor_config.h"
#include "../config/board_config.h"
#include "status_led.h"
#include "../config/balancer_config.h"
#include "logging.h"
#include "filter_manager.h"
#include <cmath>
#include <Preferences.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <Arduino.h>
#include <cstring>

namespace abbot {

static QueueHandle_t imuQueue = nullptr;
static BMI088Driver *g_driver = nullptr;
// Optional calibration queue (single-slot). When non-null the producer will also
// write each new sample into this queue (xQueueOverwrite) so a calibration routine
// can block-read exact samples without talking to the driver directly.
static QueueHandle_t calibQueue = nullptr;

// Filter instance (abstracted). Implementation chosen via compile-time config.
// Do not cache the active filter pointer here; always query the FilterManager
// to avoid dangling pointers when filters are swapped at runtime.
// Fusion configuration (mapping + params) used to transform sensor->robot axes
static fusion::FusionConfig g_fusion_cfg;
// Store sample rate (Hz) in one place so dt fallback uses the configured value
// Sample rate (Hz) used by the selected filter
static float g_filter_sample_rate_hz = 200.0f;
// Fused outputs (units: radians, radians/sec)
static float g_fused_pitch_rad = 0.0f;
static float g_fused_pitch_rate_rads = 0.0f;
static SemaphoreHandle_t g_fusion_mutex = nullptr;
// Consumer mutable state (warmup, bias, persistence) moved into a dedicated struct
static abbot::imu_consumer::ConsumerState g_consumer;
// Persistence (Preferences/NVS)
static Preferences g_prefs;
static bool g_prefs_started = false;

// Default warmup duration in seconds to seed Madgwick and gyro bias on power-up
static constexpr float kDefaultFusionWarmupSeconds = 4.0f;

// Print warmup progress (throttled). Extracted to keep imuConsumerTask compact.
static void printWarmupProgressIfAny() {
  if (g_consumer.warmup_samples_remaining > 0) {
    static uint32_t last_print = 0;
    uint32_t now = millis();
    if ((uint32_t)(now - last_print) >= 500u) {
      last_print = now;
      float secs = g_consumer.warmup_samples_remaining / g_filter_sample_rate_hz;
      char bufw[128];
      snprintf(bufw, sizeof(bufw), "FUSION: warmup remaining=%lu samples (~%.1f s)", (unsigned long)g_consumer.warmup_samples_remaining, (double)secs);
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, bufw);
    }
  }
}

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
      // Compute dt
      float dt = abbot::imu_consumer::computeDt(g_consumer, sample, g_filter_sample_rate_hz);

      // Extract raw gyro & accel
      float gx = sample.gx;
      float gy = sample.gy;
      float gz = sample.gz;

      // Warmup accumulation
      abbot::imu_consumer::accumulateWarmup(g_consumer, sample, gx, gy, gz);

      // Map sensor -> robot (bias-compensated)
      float gyro_robot[3];
      float accel_robot[3];
      float raw_g[3] = { gx, gy, gz };
      float raw_a[3] = { sample.ax, sample.ay, sample.az };
      float gyro_bias[3] = { g_consumer.gyro_bias[0], g_consumer.gyro_bias[1], g_consumer.gyro_bias[2] };
      abbot::imu_mapping::mapSensorToRobot(g_fusion_cfg, raw_g, raw_a, gyro_bias, gyro_robot, accel_robot);

      // Update selected filter (always query active filter to avoid stale pointer)
      {
        auto active = abbot::filter::getActiveFilter();
        if (active) {
          active->update(gyro_robot[0], gyro_robot[1], gyro_robot[2],
                         accel_robot[0], accel_robot[1], accel_robot[2], dt);
        }
      }

      // Finalize warmup if it just completed
      if (g_consumer.warmup_samples_remaining > 0) {
        --g_consumer.warmup_samples_remaining;
        {
          auto active = abbot::filter::getActiveFilter();
          if (active) abbot::imu_consumer::finalizeWarmupIfDone(g_consumer, *active, g_fusion_cfg);
        }
      }

      // Print warmup progress via helper to keep consumer task compact
      printWarmupProgressIfAny();

      // Visual feedback handled by status_led module
      statusLedUpdateFromConsumer(g_consumer);

      // Update bias EMA & persist when appropriate
      abbot::imu_consumer::updateBiasEmaAndPersistIfNeeded(g_consumer, sample, gyro_robot);

      // Publish fused outputs
      float fused_pitch_local = 0.0f;
      float fused_pitch_rate_local = 0.0f;
      {
        auto active = abbot::filter::getActiveFilter();
        if (active) {
          fused_pitch_local = active->getPitch();
          fused_pitch_rate_local = active->getPitchRate();
        }
      }
      abbot::imu_consumer::publishFusedOutputsUnderMutex(g_consumer, fused_pitch_local, fused_pitch_rate_local, g_fused_pitch_rad, g_fused_pitch_rate_rads, &g_fusion_mutex);

      // Read last motor commands (normalized) via motor driver API
      float left_cmd = abbot::motor::getLastMotorCommand(LEFT_MOTOR_ID);
      float right_cmd = abbot::motor::getLastMotorCommand(RIGHT_MOTOR_ID);
      abbot::imu_consumer::runBalancerCycleIfActive(fused_pitch_local, fused_pitch_rate_local, dt, left_cmd, right_cmd);

      // Emit tuning stream or capture outputs
      abbot::imu_consumer::emitTuningOrStream(g_consumer, sample, fused_pitch_local, fused_pitch_rate_local, accel_robot, gyro_robot, left_cmd, right_cmd);
      
      #if defined(ENABLE_IMU_DEBUG_LOGS)
      // Emit diagnostics if BALANCER channel enabled
      abbot::imu_consumer::emitDiagnosticsIfEnabled(sample.ts_ms, fused_pitch_local, fused_pitch_rate_local, left_cmd, right_cmd);
      #endif
      
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

  // Try to initialize BMI088 (gyro + accel)
  bool rc = driver->begin();
  if (!rc) {
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "BMI088 init failed, blinking LED red.");
    statusLedBlinkErrorLoop();
    return false;
  }


  // track whether we've requested a filter-specific warmup during init
  bool warmup_requested = false;

  // Initialize fusion mutex and default config
  if (!g_fusion_mutex) {
    g_fusion_mutex = xSemaphoreCreateMutex();
    fusion::FusionConfig cfg;
    // beta is configured in FusionConfig.h - do not override here
    // Use the BMI088 driver's configured sampling rate if available
    if (driver) {
      cfg.sample_rate = (float)driver->getSamplingHz();
    } else {
      cfg.sample_rate = 200.0f;
    }
    g_filter_sample_rate_hz = cfg.sample_rate;
    // store fusion configuration (including axis mapping) for runtime use
    g_fusion_cfg = cfg;
    // initialize filter manager which will create the default active filter
    abbot::filter::init(cfg);
  }

  // Initialize balancer controller (owns PID, deadband, persisted gains)
  abbot::balancer::controller::init();

  // Initialize logging manager and ensure TUNING channel is disabled by default
  abbot::log::init();
  abbot::log::disableChannel(abbot::log::CHANNEL_TUNING);

  // Initialize status LED (no-op if not configured)
  statusLedInit();

    // Initialize Preferences (NVS) and attempt to load persisted gyro bias via helper
  if (g_prefs.begin("abbot", false)) {
    g_prefs_started = true;
    abbot::imu_consumer::initializeConsumerStateFromPreferences(g_consumer, g_prefs);
    // If a filter preference was stored, apply it now (overrides the default)
    if (g_prefs.isKey("filter")) {
      String fn = g_prefs.getString("filter", "MADGWICK");
      if (fn.length() > 0) {
        abbot::filter::setCurrentFilterByName(fn.c_str());
        // Request filter-specific warmup if provided
        auto a = abbot::filter::getActiveFilter();
        if (a) {
          unsigned long ms = a->getWarmupDurationMs();
          if (ms > 0) {
            float s = ((float)ms) / 1000.0f;
            requestTuningWarmupSeconds(s);
            warmup_requested = true;
            char tbuf[128]; snprintf(tbuf, sizeof(tbuf), "FUSION: requested startup warmup %.3f s for %s", (double)s, fn.c_str());
            LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, tbuf);
          }
        }
      }
    }
  }

  // Log the active filter at startup so operator sees which fusion is running
  {
    const char* fname = abbot::filter::getCurrentFilterName();
    char buff[128]; snprintf(buff, sizeof(buff), "FUSION: active filter at boot=%s", fname);
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, buff);
  }
  // Also print the balancer PID gains that are active for this filter (kp/ki/kd)
  {
    float kp=0.0f, ki=0.0f, kd=0.0f;
    abbot::balancer::controller::getGains(kp, ki, kd);
    float db = abbot::balancer::controller::getDeadband();
    const char* fname = abbot::filter::getCurrentFilterName();
    char bufg[192];
    snprintf(bufg, sizeof(bufg), "FUSION: active filter=%s BALANCER gains Kp=%.4f Ki=%.4f Kd=%.4f deadband=%.4f", fname, (double)kp, (double)ki, (double)kd, (double)db);
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, bufg);
  }
  // Attempt to initialize from IMU calibration if needed (helper decides)
  abbot::imu_cal::Calibration cal;
  if (abbot::imu_cal::loadCalibration(cal)) {
    abbot::imu_consumer::initializeConsumerStateFromCalibrationIfNeeded(g_consumer, cal, g_prefs_started, &g_prefs);
  }

  // Request a short automatic warmup on power-up so filters are seeded and
  // the gyro bias accumulators are initialized before enabling the balancer.
  // If a filter-specific warmup was requested above (from persisted prefs),
  // skip the generic default to avoid overriding it.
  if (!warmup_requested) {
    requestTuningWarmupSeconds(kDefaultFusionWarmupSeconds);
  }

  // Create producer task (higher priority)
  BaseType_t r1 = xTaskCreate(imuProducerTask, "IMUProducer", 4096, driver, configMAX_PRIORITIES - 2, nullptr);
  // Create consumer task (lower priority)
  BaseType_t r2 = xTaskCreate(imuConsumerTask, "IMUConsumer", 4096, nullptr, configMAX_PRIORITIES - 3, nullptr);

  // Create serial command task for calibration UI (low priority)
  BaseType_t r3 = xTaskCreate(abbot::serialcmds::serialTaskEntry, "IMUSerial", 12288, driver, configMAX_PRIORITIES - 4, nullptr);

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

void requestTuningWarmupSeconds(float seconds) {
  abbot::imu_consumer::requestWarmup(g_consumer, seconds, g_filter_sample_rate_hz);
}

void printMadgwickDiagnostics() {
  // Acquire fusion mutex to read fused values and madgwick state
  if (g_fusion_mutex) {
    if (xSemaphoreTake(g_fusion_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      // Read quaternion and pitch/pitch rate from selected filter (if supported)
      float w=0, x=0, y=0, z=0;
      float pitch_rad = 0.0f;
      float pitch_rate = 0.0f;
      {
        auto active = abbot::filter::getActiveFilter();
        if (active) {
          active->getQuaternion(w, x, y, z);
          pitch_rad = active->getPitch();
          pitch_rate = active->getPitchRate();
        }
      }
      xSemaphoreGive(g_fusion_mutex);

      char buf[256];
      snprintf(buf, sizeof(buf), "Fusion: q=[%.6f, %.6f, %.6f, %.6f] pitch_deg=%.3f pitch_rate_deg_s=%.3f", (double)w, (double)x, (double)y, (double)z, (double)(pitch_rad * 57.295779513f), (double)(pitch_rate * 57.295779513f));
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, buf);
    } else {
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "Fusion: unable to take fusion mutex");
    }
  } else {
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "Fusion: fusion mutex not initialized");
  }

  // Print consumer warmup and gyro bias info
  // g_consumer is module-local; print key fields
  {
    char buf2[256];
    snprintf(buf2, sizeof(buf2), "Consumer: warmup_remaining=%lu warmup_total=%lu gyro_bias=[%.6f, %.6f, %.6f] bias_initialized=%d",
             (unsigned long)g_consumer.warmup_samples_remaining,
             (unsigned long)g_consumer.warmup_samples_total,
             (double)g_consumer.gyro_bias[0], (double)g_consumer.gyro_bias[1], (double)g_consumer.gyro_bias[2],
             g_consumer.gyro_bias_initialized ? 1 : 0);
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, buf2);
  }
}

bool isFusionReady() {
  // Fusion is considered ready when no warmup samples remain and gyro bias initialized
  return (g_consumer.warmup_samples_remaining == 0) && g_consumer.gyro_bias_initialized;
}

unsigned long getFusionWarmupRemaining() {
  return (unsigned long)g_consumer.warmup_samples_remaining;
}

} // namespace abbot
