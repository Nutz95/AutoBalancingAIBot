// imu_calibration.cpp
#include "imu_calibration.h"
#include <Preferences.h>
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include "SystemTasks.h"
#include "motor_control.h"
#include "motor_drivers/servo_motor_driver.h"
#include "logging.h"

namespace abbot {
namespace imu_cal {

static Calibration g_cal = {{0,0,0},{0,0,0}, 1, false};
static volatile bool g_is_calibrating = false;

// Calibration sampling interval (ms). 5 ms => 200 Hz default.
static constexpr int CALIB_SAMPLE_INTERVAL_MS = 5;

// Stability thresholds
static constexpr double GYRO_STD_THRESHOLD = 0.02; // rad/s (~1.15 deg/s)
static constexpr double ACCEL_STD_THRESHOLD = 0.2;  // m/s^2

bool loadCalibration(Calibration &out) {
  Preferences prefs;
  if (!prefs.begin("imu", true)) return false;
  size_t len = prefs.getBytesLength("cal");
  if (len != sizeof(Calibration)) {
    prefs.end();
    return false;
  }
  prefs.getBytes("cal", &out, sizeof(Calibration));
  prefs.end();
  return out.valid;
}

bool saveCalibration(const Calibration &cal) {
  Preferences prefs;
  if (!prefs.begin("imu", false)) return false;
  bool ok = prefs.putBytes("cal", &cal, sizeof(Calibration)) == sizeof(Calibration);
  prefs.end();
  return ok;
}

void applyCalibrationToSample(struct abbot::IMUSample &s) {
  if (!g_cal.valid) return;
  // NOTE: gyro bias is applied at runtime by the balancer task (abbot namespace)
  // to support warm-up estimation and EMA refinement. Do NOT subtract
  // `g_cal.gyro_bias[]` here to avoid double-application when the runtime
  // bias is also in use.
  s.ax -= g_cal.accel_offset[0];
  s.ay -= g_cal.accel_offset[1];
  s.az -= g_cal.accel_offset[2];
}

static void computeMeanStd(const float *samples, int n, double &mean, double &std) {
  double sum = 0.0;
  for (int i=0;i<n;i++) sum += samples[i];
  mean = sum / n;
  double s = 0.0;
  for (int i=0;i<n;i++) s += (samples[i]-mean)*(samples[i]-mean);
  std = (n>1) ? sqrt(s/(n-1)) : 0.0;
}

bool startGyroCalibration(class abbot::BMI088Driver &driver, int nSamples) {
  if (nSamples <= 0) return false;
  g_is_calibrating = true;
  LOG_PRINTF(abbot::log::CHANNEL_IMU, "CALIB START GYRO: sampling %d samples...\n", nSamples);
  unsigned long startMs = millis();
  LOG_PRINTF(abbot::log::CHANNEL_IMU, "CALIB TIME START ms=%lu\n", startMs);

  // Use Welford's online algorithm to compute mean and variance without large allocations
  double mean_x = 0.0, mean_y = 0.0, mean_z = 0.0;
  double m2_x = 0.0, m2_y = 0.0, m2_z = 0.0;
  int count = 0;
  abbot::IMUSample sample;
  // Progress reporting: print every ~1% (or at least once per sample if nSamples<100)
  int progressInterval = nSamples / 100;
  if (progressInterval < 1) progressInterval = 1;
  // Create a single-slot queue and attach it to the IMU producer so we receive
  // each new sample as it's produced (no direct driver reads from this task).
  QueueHandle_t q = xQueueCreate(1, sizeof(abbot::IMUSample));
  if (!q) {
    LOG_PRINTLN(abbot::log::CHANNEL_IMU, "CALIB FAIL: cannot create queue");
    g_is_calibrating = false;
    return false;
  }
  attachCalibrationQueue(q);

  bool motor_was_enabled = abbot::motor_control::areMotorsEnabled();
  if (motor_was_enabled) {
    abbot::motor_control::disableMotors();
  }

  while (count < nSamples) {
    if (xQueueReceive(q, &sample, portMAX_DELAY) == pdTRUE) {
      // gx, gy, gz are in rad/s
      double dx = sample.gx;
      double dy = sample.gy;
      double dz = sample.gz;
      count++;
      double delta = dx - mean_x; mean_x += delta / count; m2_x += delta*(dx - mean_x);
      delta = dy - mean_y; mean_y += delta / count; m2_y += delta*(dy - mean_y);
      delta = dz - mean_z; mean_z += delta / count; m2_z += delta*(dz - mean_z);
      if ((count % progressInterval) == 0) {
        int pct = (count * 100) / nSamples;
        if (pct > 100) pct = 100;
        LOG_PRINTF(abbot::log::CHANNEL_IMU, "CALIB PROGRESS GYRO %d%% (%d/%d)\n", pct, count, nSamples);
      }
    } else {
      // unexpected timeout; abort
      LOG_PRINTLN(abbot::log::CHANNEL_IMU, "CALIB FAIL: queue timeout");
      detachCalibrationQueue();
      vQueueDelete(q);
      if (motor_was_enabled) abbot::motor_control::enableMotors();
      g_is_calibrating = false;
      return false;
    }
  }
  // compute std
  double std_x = (count > 1) ? sqrt(m2_x / (count - 1)) : 0.0;
  double std_y = (count > 1) ? sqrt(m2_y / (count - 1)) : 0.0;
  double std_z = (count > 1) ? sqrt(m2_z / (count - 1)) : 0.0;

  if (std_x > GYRO_STD_THRESHOLD || std_y > GYRO_STD_THRESHOLD || std_z > GYRO_STD_THRESHOLD) {
    LOG_PRINTF(abbot::log::CHANNEL_IMU, "CALIB FAIL gyro unstable: std=(%.6f,%.6f,%.6f)\n", std_x, std_y, std_z);
    unsigned long endMs = millis();
    LOG_PRINTF(abbot::log::CHANNEL_IMU, "CALIB TIME END ms=%lu elapsed_ms=%lu\n", endMs, (endMs - startMs));
    detachCalibrationQueue();
    vQueueDelete(q);
    if (motor_was_enabled) abbot::motor_control::enableMotors();
    g_is_calibrating = false;
    return false;
  }

  g_cal.gyro_bias[0] = (float)mean_x;
  g_cal.gyro_bias[1] = (float)mean_y;
  g_cal.gyro_bias[2] = (float)mean_z;
  g_cal.valid = true;
  g_cal.version++;
  bool saved = saveCalibration(g_cal);
  LOG_PRINTF(abbot::log::CHANNEL_IMU, "CALIB DONE gyro_bias=%.6f,%.6f,%.6f\n", g_cal.gyro_bias[0], g_cal.gyro_bias[1], g_cal.gyro_bias[2]);
  unsigned long endMs = millis();
  LOG_PRINTF(abbot::log::CHANNEL_IMU, "CALIB TIME END ms=%lu elapsed_ms=%lu\n", endMs, (endMs - startMs));
  detachCalibrationQueue();
  vQueueDelete(q);
  if (motor_was_enabled) abbot::motor_control::enableMotors();
  g_is_calibrating = false;
  return saved;
}

bool startAccelCalibration(class abbot::BMI088Driver &driver, int nSamples) {
  if (nSamples <= 0) return false;
  g_is_calibrating = true;
  LOG_PRINTF(abbot::log::CHANNEL_IMU, "CALIB START ACCEL: sampling %d samples...\n", nSamples);
  unsigned long startMs = millis();
  LOG_PRINTF(abbot::log::CHANNEL_IMU, "CALIB TIME START ms=%lu\n", startMs);

  // Online mean/variance for accel axes
  double mean_x = 0.0, mean_y = 0.0, mean_z = 0.0;
  double m2_x = 0.0, m2_y = 0.0, m2_z = 0.0;
  int count = 0;
  abbot::IMUSample sample;
  // Create a single-slot queue and attach it to the IMU producer so we receive
  // each new sample as it's produced (no direct driver reads from this task).
  QueueHandle_t q = xQueueCreate(1, sizeof(abbot::IMUSample));
  if (!q) {
    LOG_PRINTLN(abbot::log::CHANNEL_IMU, "CALIB FAIL: cannot create queue");
    g_is_calibrating = false;
    return false;
  }
  attachCalibrationQueue(q);

  bool motor_was_enabled = abbot::motor_control::areMotorsEnabled();
  if (motor_was_enabled) {
    abbot::motor_control::disableMotors();
  }

  while (count < nSamples) {
    if (xQueueReceive(q, &sample, portMAX_DELAY) == pdTRUE) {
      double ax = sample.ax;
      double ay = sample.ay;
      double az = sample.az;
      count++;
      double delta = ax - mean_x; mean_x += delta / count; m2_x += delta*(ax - mean_x);
      delta = ay - mean_y; mean_y += delta / count; m2_y += delta*(ay - mean_y);
      delta = az - mean_z; mean_z += delta / count; m2_z += delta*(az - mean_z);
      // Progress reporting for accel
      int progressIntervalA = nSamples / 100;
      if (progressIntervalA < 1) progressIntervalA = 1;
      if ((count % progressIntervalA) == 0) {
        int pct = (count * 100) / nSamples;
        if (pct > 100) pct = 100;
        LOG_PRINTF(abbot::log::CHANNEL_IMU, "CALIB PROGRESS ACCEL %d%% (%d/%d)\n", pct, count, nSamples);
      }
    } else {
      LOG_PRINTLN(abbot::log::CHANNEL_IMU, "CALIB FAIL: queue timeout");
      detachCalibrationQueue();
      vQueueDelete(q);
      if (motor_was_enabled) abbot::motor_control::enableMotors();
      g_is_calibrating = false;
      return false;
    }
  }

  // compute std
  double std_x = (count > 1) ? sqrt(m2_x / (count - 1)) : 0.0;
  double std_y = (count > 1) ? sqrt(m2_y / (count - 1)) : 0.0;
  double std_z = (count > 1) ? sqrt(m2_z / (count - 1)) : 0.0;

  if (std_x > ACCEL_STD_THRESHOLD || std_y > ACCEL_STD_THRESHOLD || std_z > ACCEL_STD_THRESHOLD) {
    LOG_PRINTF(abbot::log::CHANNEL_IMU, "CALIB FAIL accel unstable: std=(%.6f,%.6f,%.6f)\n", std_x, std_y, std_z);
    unsigned long endMs = millis();
    LOG_PRINTF(abbot::log::CHANNEL_IMU, "CALIB TIME END ms=%lu elapsed_ms=%lu\n", endMs, (endMs - startMs));
    detachCalibrationQueue();
    vQueueDelete(q);
    if (motor_was_enabled) abbot::motor_control::enableMotors();
    g_is_calibrating = false;
    return false;
  }

  // For a single-position calibration, expected gravity vector depends on orientation.
  // Here we assume Z-up (common case): expected g = +9.80665 m/s^2 on Z.
  g_cal.accel_offset[0] = (float)(mean_x - 0.0);
  g_cal.accel_offset[1] = (float)(mean_y - 0.0);
  g_cal.accel_offset[2] = (float)(mean_z - 9.80665);

  g_cal.valid = true;
  g_cal.version++;
  bool saved = saveCalibration(g_cal);
  LOG_PRINTF(abbot::log::CHANNEL_IMU, "CALIB DONE accel_offset=%.6f,%.6f,%.6f\n", g_cal.accel_offset[0], g_cal.accel_offset[1], g_cal.accel_offset[2]);
  unsigned long endMs = millis();
  LOG_PRINTF(abbot::log::CHANNEL_IMU, "CALIB TIME END ms=%lu elapsed_ms=%lu\n", endMs, (endMs - startMs));
  detachCalibrationQueue();
  vQueueDelete(q);
  if (motor_was_enabled) abbot::motor_control::enableMotors();
  g_is_calibrating = false;
  return saved;
}

void dumpCalibration() {
  if (!g_cal.valid) {
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "CALIB: none");
    return;
  }
  LOG_PRINTF(abbot::log::CHANNEL_DEFAULT, "CALIB DUMP gyro_bias=%.6f,%.6f,%.6f\n", g_cal.gyro_bias[0], g_cal.gyro_bias[1], g_cal.gyro_bias[2]);
  LOG_PRINTF(abbot::log::CHANNEL_DEFAULT, "CALIB DUMP accel_offset=%.6f,%.6f,%.6f\n", g_cal.accel_offset[0], g_cal.accel_offset[1], g_cal.accel_offset[2]);
}

void resetCalibration() {
  Preferences prefs;
  if (prefs.begin("imu", false)) {
    prefs.remove("cal");
    prefs.end();
  }
  g_cal = {{0,0,0},{0,0,0},1,false};
  LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "CALIB RESET");
}

bool isCalibrating() { return g_is_calibrating; }

void installCalibration(const Calibration &cal) {
  g_cal = cal;
}

// Serial command processing was moved to serial_commands.cpp

} // namespace imu_cal
} // namespace abbot
