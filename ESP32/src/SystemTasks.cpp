// SystemTasks.cpp
#include "SystemTasks.h"
#include "BMI088Driver.h"
#include "imu_calibration.h"
#include "serial_commands.h"
#include "imu_fusion.h"
#include "motor_driver.h"
#include "../config/motor_config.h"
#include "logging.h"
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

      // Update Madgwick filter with sensor sample (gx,gy,gz rad/s ; ax,ay,az m/s^2)
      g_madgwick.update(sample.gx, sample.gy, sample.gz, sample.ax, sample.ay, sample.az, dt);

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

      // If tuning stream is active, print CSV line with both degrees and radians:
      // timestamp_ms,pitch_deg,pitch_rad,pitch_rate_deg,pitch_rate_rad,left_cmd,right_cmd
      if (abbot::log::isChannelEnabled(abbot::log::CHANNEL_TUNING)) {
        const float RAD2DEG = 57.29577951308232f; // 180/pi
        float pitch_deg = fused_pitch_local * RAD2DEG;
        float pitch_rate_dps = fused_pitch_rate_local * RAD2DEG;
        float pitch_rad = fused_pitch_local;
        float pitch_rate_rads = fused_pitch_rate_local;
        // Read last motor commands (normalized) via motor driver API
        float left_cmd = abbot::motor::getLastMotorCommand(LEFT_MOTOR_ID);
        float right_cmd = abbot::motor::getLastMotorCommand(RIGHT_MOTOR_ID);
        // Print CSV line (use LOG_PRINTF so it's formatted under the logging mutex)
        LOG_PRINTF(abbot::log::CHANNEL_TUNING,
                   "%lu,%.4f,%.6f,%.4f,%.6f,%.4f,%.4f\n",
                   sample.ts_ms,
                   pitch_deg,
                   pitch_rad,
                   pitch_rate_dps,
                   pitch_rate_rads,
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
  if (!driver) return false;
  g_driver = driver;
  if (!imuQueue) {
    imuQueue = xQueueCreate(1, sizeof(IMUSample));
    if (!imuQueue) return false;
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
  // Print CSV header for consumers
  LOG_PRINTLN(abbot::log::CHANNEL_TUNING, "timestamp_ms,pitch_deg,pitch_rad,pitch_rate_deg,pitch_rate_rad,left_cmd,right_cmd");
}

void stopTuningStream() {
  abbot::log::disableChannel(abbot::log::CHANNEL_TUNING);
}

bool isTuningStreamActive() {
  return abbot::log::isChannelEnabled(abbot::log::CHANNEL_TUNING);
}

} // namespace abbot
