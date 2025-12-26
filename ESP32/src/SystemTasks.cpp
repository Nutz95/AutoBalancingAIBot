#include "units.h"
// SystemTasks.cpp
#include "../config/balancer_config.h"
#include "../config/board_config.h"
#include "../config/imu_filter_config.h"
#include "../config/motor_configs/servo_motor_config.h"
#include "../include/balancer_controller.h"
#include "../include/esp_wifi_console.h"
#include "BMI088Driver.h"
#include "SystemTasks.h"
#include "filter_manager.h"
#include "imu_calibration.h"
#include "imu_consumer_helpers.h"
#include "imu_filter.h"
#include "imu_fusion.h"
#include "imu_mapping.h"
#include "logging.h"
#include "motor_drivers/driver_manager.h"
#include "serial_commands.h"
#include "status_led.h"
#include "tuning_capture.h"
#include <Arduino.h>
#include <Preferences.h>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

namespace abbot {

static QueueHandle_t imuQueue = nullptr;
static BMI088Driver *g_driver = nullptr;
// Optional calibration queue (single-slot). When non-null the producer will
// also write each new sample into this queue (xQueueOverwrite) so a calibration
// routine can block-read exact samples without talking to the driver directly.
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
// Consumer mutable state (warmup, bias, persistence) moved into a dedicated
// struct
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
      float secs =
          g_consumer.warmup_samples_remaining / g_filter_sample_rate_hz;
      char bufw[128];
      snprintf(
          bufw, sizeof(bufw), "FUSION: warmup remaining=%lu samples (~%.1f s)",
          (unsigned long)g_consumer.warmup_samples_remaining, (double)secs);
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, bufw);
    }
  }
}

static void imuProducerTask(void *pvParameters) {
  BMI088Driver *driver = reinterpret_cast<BMI088Driver *>(pvParameters);
  IMUSample sample;
  for (;;) {
    // Attempt to read; BMI088Driver::read will enforce sampling interval
    if (driver->read(sample)) {
      // Single-slot overwrite keeps only the latest sample
      if (imuQueue) {
        xQueueOverwrite(imuQueue, &sample);
      }
      // If a calibration queue is attached, push sample there as well
      // (non-blocking)
      if (calibQueue) {
        xQueueOverwrite(calibQueue, &sample);
      }
    }
    // Short delay to yield; real timing enforced by driver->read
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

static void wifiConsoleTask(void *pvParameters) {
  (void)pvParameters;
  for (;;) {
    abbot::wifi_console::loop();
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

// imuConsumerTask
// Responsibilities:
// - Receive IMU samples from `imuQueue`.
// - Measure and log the effective IMU sampling frequency.
// - Map sensor-frame data into robot frame and apply gyro bias.
// - Update the active filter, handle warmup, and publish fused outputs.
// - Maintain gyro bias EMA, persist when stable, and drive the balancer cycle.
// Helpers used: measureAndLogImuFrequency, mapSensorToRobotFrame,
// finalizeWarmupIfDone, updateBiasEmaAndPersistIfNeeded, emitTuningOrStream,
// emitImuDebugLogsIfEnabled.
static void imuConsumerTask(void *pvParameters) {
  (void)pvParameters;
  IMUSample sample;
  uint32_t last_debug_print_ms = 0;
  
  // IMU frequency measurement state
  abbot::imu_consumer::ImuFrequencyMeasurement freq_measurement;
  
  for (;;) {
    if (!imuQueue || xQueueReceive(imuQueue, &sample, portMAX_DELAY) != pdTRUE) {
      continue;
    }
    
    // Measure and log actual IMU frequency
    abbot::imu_consumer::measureAndLogImuFrequency(freq_measurement,
                                                   g_filter_sample_rate_hz);
    
    // Compute dt from sample timestamps
    float dt = abbot::imu_consumer::computeDt(g_consumer, sample,
                                              g_filter_sample_rate_hz);

    // Accumulate warmup samples (gyro raw values)
    abbot::imu_consumer::accumulateWarmup(g_consumer, sample,
                                          sample.gx, sample.gy, sample.gz);

    // Map sensor frame -> robot frame (with gyro bias compensation)
    float gyro_robot[3];
    float accel_robot[3];
    abbot::imu_consumer::mapSensorToRobotFrame(g_fusion_cfg, sample,
                                               g_consumer.gyro_bias,
                                               gyro_robot, accel_robot);

    // Update active filter with mapped data
    {
      auto active = abbot::filter::getActiveFilter();
      if (active) {
        active->update(gyro_robot[0], gyro_robot[1], gyro_robot[2],
                       accel_robot[0], accel_robot[1], accel_robot[2], dt);
      }
    }

    // Process warmup phase (decrement counter, finalize if done)
    if (g_consumer.warmup_samples_remaining > 0) {
      --g_consumer.warmup_samples_remaining;
      auto active = abbot::filter::getActiveFilter();
      if (active) {
        abbot::imu_consumer::finalizeWarmupIfDone(g_consumer, *active,
                                                  g_fusion_cfg);
      }
    }
    printWarmupProgressIfAny();
    statusLedUpdateFromConsumer(g_consumer);

    // Update gyro bias EMA and persist when appropriate
    abbot::imu_consumer::updateBiasEmaAndPersistIfNeeded(g_consumer, sample,
                                                         gyro_robot);

    // Get fused pitch outputs from active filter
    float fused_pitch_local = 0.0f;
    float fused_pitch_rate_local = 0.0f;
    {
      auto active = abbot::filter::getActiveFilter();
      if (active) {
        fused_pitch_local = active->getPitch();
        fused_pitch_rate_local = active->getPitchRate();
      }
    }

    // Publish fused outputs under mutex for other tasks
    abbot::imu_consumer::publishFusedOutputsUnderMutex(
        g_consumer, fused_pitch_local, fused_pitch_rate_local,
        g_fused_pitch_rad, g_fused_pitch_rate_rads, &g_fusion_mutex);

    // Get current motor commands for logging/telemetry
    float left_cmd = 0.0f;
    float right_cmd = 0.0f;
    abbot::imu_consumer::getLastMotorCommands(left_cmd, right_cmd);

    // Run balancer control cycle if active
    abbot::imu_consumer::runBalancerCycleIfActive(
        fused_pitch_local, fused_pitch_rate_local, dt, left_cmd, right_cmd);

    // Emit tuning stream or capture outputs
    abbot::imu_consumer::emitTuningOrStream(
        g_consumer, sample, fused_pitch_local, fused_pitch_rate_local,
        accel_robot, gyro_robot, left_cmd, right_cmd);

#if defined(ENABLE_IMU_DEBUG_LOGS)
    abbot::imu_consumer::emitDiagnosticsIfEnabled(
        sample.ts_ms, fused_pitch_local, fused_pitch_rate_local,
        left_cmd, right_cmd);
#endif

    // Emit raw IMU debug logs (throttled)
    abbot::imu_consumer::emitImuDebugLogsIfEnabled(sample, last_debug_print_ms);
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
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT,
                "BMI088 init failed, blinking LED red.");
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

  // Start optional Wi-Fi console module (no-op if disabled)
  abbot::wifi_console::begin();

  // Initialize Preferences (NVS) and attempt to load persisted gyro bias via
  // helper
  if (g_prefs.begin("abbot", false)) {
    g_prefs_started = true;
    abbot::imu_consumer::initializeConsumerStateFromPreferences(g_consumer,
                                                                g_prefs);
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
            char tbuf[128];
            snprintf(tbuf, sizeof(tbuf),
                     "FUSION: requested startup warmup %.3f s for %s",
                     (double)s, fn.c_str());
            LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, tbuf);
          }
        }
      }
    }
  }

  // Log the active filter at startup so operator sees which fusion is running
  {
    const char *fname = abbot::filter::getCurrentFilterName();
    char buff[128];
    snprintf(buff, sizeof(buff), "FUSION: active filter at boot=%s", fname);
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, buff);
  }
  // Also print the balancer PID gains that are active for this filter
  // (kp/ki/kd)
  {
    float kp = 0.0f, ki = 0.0f, kd = 0.0f;
    abbot::balancer::controller::getGains(kp, ki, kd);
    float db = abbot::balancer::controller::getDeadband();
    const char *fname = abbot::filter::getCurrentFilterName();
    char bufg[192];
    snprintf(bufg, sizeof(bufg),
             "FUSION: active filter=%s BALANCER gains Kp=%.4f Ki=%.4f Kd=%.4f "
             "deadband=%.4f",
             fname, (double)kp, (double)ki, (double)kd, (double)db);
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, bufg);
  }
  // Attempt to initialize from IMU calibration if needed (helper decides)
  abbot::imu_cal::Calibration cal;
  if (abbot::imu_cal::loadCalibration(cal)) {
    abbot::imu_consumer::initializeConsumerStateFromCalibrationIfNeeded(
        g_consumer, cal, g_prefs_started, &g_prefs);
  }

  // Request a short automatic warmup on power-up so filters are seeded and
  // the gyro bias accumulators are initialized before enabling the balancer.
  // If a filter-specific warmup was requested above (from persisted prefs),
  // skip the generic default to avoid overriding it.
  if (!warmup_requested) {
    requestTuningWarmupSeconds(kDefaultFusionWarmupSeconds);
  }

  // Create producer task (higher priority)
  BaseType_t r1 = xTaskCreate(imuProducerTask, "IMUProducer", 4096, driver,
                              configMAX_PRIORITIES - 2, nullptr);
  // Create consumer task (lower priority)
  BaseType_t r2 = xTaskCreate(imuConsumerTask, "IMUConsumer", 4096, nullptr,
                              configMAX_PRIORITIES - 3, nullptr);

  // Create serial command task for calibration UI (low priority)
  BaseType_t r3 = xTaskCreate(abbot::serialcmds::serialTaskEntry, "IMUSerial",
                              12288, driver, configMAX_PRIORITIES - 4, nullptr);

  // Create a small background task to service the Wi‑Fi console (drain queue,
  // accept clients, handle incoming lines). Low priority so it won't interfere
  // with IMU timing.
  // Wi‑Fi console uses moderate stack (WiFi library + small buffers). Allocate
  // more stack to avoid watchpoint / canary failures on some chips.
  BaseType_t r4 = xTaskCreate(wifiConsoleTask, "WiFiConsole", 8192, nullptr,
                              configMAX_PRIORITIES - 5, nullptr);

  return (r1 == pdPASS) && (r2 == pdPASS) && (r3 == pdPASS) && (r4 == pdPASS);
}

void attachCalibrationQueue(QueueHandle_t q) { calibQueue = q; }

void detachCalibrationQueue() { calibQueue = nullptr; }

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
  abbot::imu_consumer::requestWarmup(g_consumer, seconds,
                                     g_filter_sample_rate_hz);
}

void printMadgwickDiagnostics() {
  // Acquire fusion mutex to read fused values and madgwick state
  if (g_fusion_mutex) {
    if (xSemaphoreTake(g_fusion_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      // Read quaternion and pitch/pitch rate from selected filter (if
      // supported)
      float w = 0, x = 0, y = 0, z = 0;
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
      snprintf(buf, sizeof(buf),
               "Fusion: q=[%.6f, %.6f, %.6f, %.6f] pitch_deg=%.3f "
               "pitch_rate_deg_s=%.3f",
               (double)w, (double)x, (double)y, (double)z,
               (double)(pitch_rad * 57.295779513f),
               (double)(pitch_rate * 57.295779513f));
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, buf);
    } else {
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT,
                  "Fusion: unable to take fusion mutex");
    }
  } else {
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT,
                "Fusion: fusion mutex not initialized");
  }

  // Print consumer warmup and gyro bias info
  // g_consumer is module-local; print key fields
  {
    char buf2[256];
    snprintf(buf2, sizeof(buf2),
             "Consumer: warmup_remaining=%lu warmup_total=%lu gyro_bias=[%.6f, "
             "%.6f, %.6f] bias_initialized=%d",
             (unsigned long)g_consumer.warmup_samples_remaining,
             (unsigned long)g_consumer.warmup_samples_total,
             (double)g_consumer.gyro_bias[0], (double)g_consumer.gyro_bias[1],
             (double)g_consumer.gyro_bias[2],
             g_consumer.gyro_bias_initialized ? 1 : 0);
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, buf2);
  }
}

bool isFusionReady() {
  // Fusion is considered ready when no warmup samples remain and gyro bias
  // initialized
  return (g_consumer.warmup_samples_remaining == 0) &&
         g_consumer.gyro_bias_initialized;
}

unsigned long getFusionWarmupRemaining() {
  return (unsigned long)g_consumer.warmup_samples_remaining;
}

} // namespace abbot
