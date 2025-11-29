#include "units.h"
// SystemTasks.cpp
#include "SystemTasks.h"
#include "BMI088Driver.h"
#include "imu_calibration.h"
#include "serial_commands.h"
#include "imu_fusion.h"
#include "imu_mapping.h"
#include "imu_consumer_helpers.h"
#include "motor_driver.h"
#include "tuning_capture.h"
#include "../include/balancer_controller.h"
#include "../config/motor_config.h"
#include "../config/balancer_config.h"
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
static fusion::Madgwick g_madgwick;
// Fusion configuration (mapping + params) used to transform sensor->robot axes
static fusion::FusionConfig g_fusion_cfg;
// Store sample rate (Hz) in one place so dt fallback uses the configured value
static float g_madgwick_sample_rate_hz = 200.0f;
// Fused outputs (units: radians, radians/sec)
static float g_fused_pitch_rad = 0.0f;
static float g_fused_pitch_rate_rads = 0.0f;
static SemaphoreHandle_t g_fusion_mutex = nullptr;
// Consumer mutable state (warmup, bias, persistence) moved into a dedicated struct
static abbot::imu_consumer::ConsumerState g_consumer;
// Persistence (Preferences/NVS)
static Preferences g_prefs;
static bool g_prefs_started = false;
// Note: tuning stream state is now managed by the logging channel manager

// Helpers moved to `imu_consumer_helpers` (see include above)

// Sensor->robot mapping and bias compensation is implemented in
// `imu_mapping` (include/imu_mapping.h, src/imu_mapping.cpp).

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
      float dt = abbot::imu_consumer::computeDt(g_consumer, sample, g_madgwick_sample_rate_hz);

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

      // Update Madgwick
      g_madgwick.update(gyro_robot[0], gyro_robot[1], gyro_robot[2],
                        accel_robot[0], accel_robot[1], accel_robot[2], dt);

      // Finalize warmup if it just completed
      if (g_consumer.warmup_samples_remaining > 0) {
        --g_consumer.warmup_samples_remaining;
        abbot::imu_consumer::finalizeWarmupIfDone(g_consumer, g_madgwick, g_fusion_cfg);
      }

      // Update bias EMA & persist when appropriate
      abbot::imu_consumer::updateBiasEmaAndPersistIfNeeded(g_consumer, sample, gyro_robot);

      // Publish fused outputs
      float fused_pitch_local = g_madgwick.getPitch();
      float fused_pitch_rate_local = g_madgwick.getPitchRate();
      abbot::imu_consumer::publishFusedOutputsUnderMutex(g_consumer, fused_pitch_local, fused_pitch_rate_local, g_fused_pitch_rad, g_fused_pitch_rate_rads, &g_fusion_mutex);

      // Read last motor commands (normalized) via motor driver API
      float left_cmd = abbot::motor::getLastMotorCommand(LEFT_MOTOR_ID);
      float right_cmd = abbot::motor::getLastMotorCommand(RIGHT_MOTOR_ID);
      abbot::imu_consumer::runBalancerCycleIfActive(fused_pitch_local, fused_pitch_rate_local, dt, left_cmd, right_cmd);

      // Emit tuning stream or capture outputs
      abbot::imu_consumer::emitTuningOrStream(g_consumer, sample, fused_pitch_local, fused_pitch_rate_local, accel_robot, gyro_robot, left_cmd, right_cmd);
      // Emit diagnostics if BALANCER channel enabled
      abbot::imu_consumer::emitDiagnosticsIfEnabled(sample.ts_ms, fused_pitch_local, fused_pitch_rate_local, left_cmd, right_cmd);
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
    // store fusion configuration (including axis mapping) for runtime use
    g_fusion_cfg = cfg;
    g_madgwick.begin(cfg);
  }

  // Initialize balancer controller (owns PID, deadband, persisted gains)
  abbot::balancer::controller::init();
  // g_balancer_active is managed by the controller

  // Initialize logging manager and ensure TUNING channel is disabled by default
  abbot::log::init();
  abbot::log::disableChannel(abbot::log::CHANNEL_TUNING);

  // Initialize Preferences (NVS) and attempt to load persisted gyro bias via helper
  if (g_prefs.begin("abbot", false)) {
    g_prefs_started = true;
    abbot::imu_consumer::initializeConsumerStateFromPreferences(g_consumer, g_prefs);
  }
  // Attempt to initialize from IMU calibration if needed (helper decides)
  abbot::imu_cal::Calibration cal;
  if (abbot::imu_cal::loadCalibration(cal)) {
    abbot::imu_consumer::initializeConsumerStateFromCalibrationIfNeeded(g_consumer, cal, g_prefs_started, &g_prefs);
  }

  // Create producer task (higher priority)
  BaseType_t r1 = xTaskCreate(imuProducerTask, "IMUProducer", 4096, driver, configMAX_PRIORITIES - 2, nullptr);
  // Create consumer task (lower priority)
  BaseType_t r2 = xTaskCreate(imuConsumerTask, "IMUConsumer", 4096, nullptr, configMAX_PRIORITIES - 3, nullptr);

  // Create serial command task for calibration UI (low priority)
  BaseType_t r3 = xTaskCreate(abbot::serialcmds::serialTaskEntry, "IMUSerial", 4096, driver, configMAX_PRIORITIES - 4, nullptr);

  return (r1 == pdPASS) && (r2 == pdPASS) && (r3 == pdPASS);
}

void startBalancer() {
  abbot::balancer::controller::start(abbot::getFusedPitch());
}

void stopBalancer() {
  abbot::balancer::controller::stop();
}

bool isBalancerActive() {
  return abbot::balancer::controller::isActive();
}

void setBalancerGains(float kp, float ki, float kd) {
  abbot::balancer::controller::setGains(kp, ki, kd);
}

void getBalancerGains(float &kp, float &ki, float &kd) {
  abbot::balancer::controller::getGains(kp, ki, kd);
}

float getBalancerDeadband() {
  return abbot::balancer::controller::getDeadband();
}

void setBalancerDeadband(float db) {
  abbot::balancer::controller::setDeadband(db);
}

void calibrateBalancerDeadband() {
  abbot::balancer::controller::calibrateDeadband();
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
  abbot::imu_consumer::requestWarmup(g_consumer, seconds, g_madgwick_sample_rate_hz);
}

} // namespace abbot
