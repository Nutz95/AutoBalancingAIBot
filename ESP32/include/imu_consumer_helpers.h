#pragma once
#include "imu_drivers/IIMUDriver.h"
#include "imu_calibration.h"
#include "imu_filter.h"
#include "imu_fusion.h"
#include <Preferences.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

namespace abbot {
namespace imu_consumer {

/**
 * IMU frequency measurement state used by the consumer to accumulate
 * samples and periodically log the measured sampling rate.
 */
struct ImuFrequencyMeasurement {
  uint32_t sample_count = 0;
  uint32_t start_ms = 0;
  static constexpr uint32_t kLogIntervalMs = 2000;  // Log every 2 seconds
};

// Mutable state used by the consumer helpers. Designed to be passed by
// reference so helpers do not directly depend on SystemTasks globals.
struct ConsumerState {
  unsigned long last_sample_timestamp_us = 0;
  // warmup counters and accumulators
  uint32_t warmup_samples_remaining = 0;
  uint32_t warmup_samples_total = 0;
  float warm_ax_sum = 0.0f;
  float warm_ay_sum = 0.0f;
  float warm_az_sum = 0.0f;
  float warm_gx_sum = 0.0f;
  float warm_gy_sum = 0.0f;
  float warm_gz_sum = 0.0f;
  // runtime gyro bias (rad/s)
  float gyro_bias[3] = {0.0f, 0.0f, 0.0f};
  bool gyro_bias_initialized = false;
  // EMA and persistence parameters (tunable)
  float bias_ema_alpha = 0.01f;
  uint32_t last_persist_ms = 0;
  uint32_t persist_interval_ms = 60000;
  float persist_delta_thresh = 0.01f;
  bool pending_initial_persist = false;
  uint32_t initial_persist_deadline_ms = 0;
  uint32_t initial_persist_timeout_ms = 5000;
  float initial_persist_candidate[3] = {0.0f, 0.0f, 0.0f};
  // Preferences pointer (may be null if NVS unavailable)
  Preferences *prefs = nullptr;
  bool prefs_started = false;
};

// Compute dt (seconds) and update state's last timestamp
float computeDt(ConsumerState &state, const IMUSample &sample,
                float sample_rate_hz);

// Accumulate warmup sums if warmup requested
void accumulateWarmup(ConsumerState &state, const IMUSample &sample, float gx,
                      float gy, float gz);

// If warmup finished, seed Madgwick from average accel and mark bias
// initialized
void finalizeWarmupIfDone(ConsumerState &state, abbot::IMUFilter &filter,
                          const fusion::FusionConfig &cfg);

// Update gyro bias EMA when stationary and persist into NVS when conditions met
void updateBiasEmaAndPersistIfNeeded(ConsumerState &state,
                                     const IMUSample &sample,
                                     const float gyro_robot[3]);

// Publish fused outputs under a provided mutex & storage locations
void publishFusedOutputsUnderMutex(ConsumerState &state,
                                   float fused_pitch_local,
                                   float fused_pitch_rate_local,
                                   float &out_fused_pitch_rad,
                                   float &out_fused_pitch_rate_rads,
                                   SemaphoreHandle_t *fusion_mutex);

// Run balancer cycle (calls controller) and refresh last motor commands.
// Pass accel/gyro in robot frame (m/s^2, rad/s) for logging.
void runBalancerCycleIfActive(float fused_pitch_local,
                              float fused_pitch_rate_local, float dt,
                              const float accel_robot[3],
                              const float gyro_robot[3],
                              float &left_cmd, float &right_cmd);

// Emit tuning/capture output (CSV or capture submit)
void emitTuningOrStream(const ConsumerState &state, const IMUSample &sample,
                        float fused_pitch_local, float fused_pitch_rate_local,
                        const float accel_robot[3], const float gyro_robot[3],
                        float left_cmd, float right_cmd);

// --- New Initialization & Diagnostics Helpers ---
// Initialize consumer state from Preferences (loads persisted gyro bias if
// present).
void initializeConsumerStateFromPreferences(ConsumerState &state,
                                            Preferences &prefs);
// Initialize consumer state gyro bias from IMU calibration if Preferences
// missing or zero.
void initializeConsumerStateFromCalibrationIfNeeded(
    ConsumerState &state, const abbot::imu_cal::Calibration &cal,
    bool prefs_started, Preferences *prefs);
// Request warmup period (seconds) converting to sample count via provided
// sample rate.
void requestWarmup(ConsumerState &state, float seconds, float sample_rate_hz);
// Emit balancer diagnostics if BALANCER channel enabled.
void emitDiagnosticsIfEnabled(uint32_t ts_ms, float fused_pitch_local,
                              float fused_pitch_rate_local, float left_cmd,
                              float right_cmd, const float accel_robot[3], 
                              const float gyro_robot[3], float freq_hz, uint32_t lat_us);

/**
 * Measure and log IMU frequency.
 * Call this once for each sample received. The helper will accumulate
 * counts and emit a single log line every `ImuFrequencyMeasurement::kLogIntervalMs`.
 * @param freq_state mutable state used to accumulate samples and timing
 * @param target_hz the configured target sampling rate (for percent calc)
 * @return true if a log message was emitted during this call
 */
bool measureAndLogImuFrequency(ImuFrequencyMeasurement &freq_state,
                               float target_hz);

/**
 * Map sensor-frame measurements from `sample` into the robot frame using
 * `cfg` axis mapping and apply `gyro_bias` compensation.
 * Outputs are written to `gyro_robot` and `accel_robot` arrays (size 3).
 */
void mapSensorToRobotFrame(const fusion::FusionConfig &cfg,
                           const IMUSample &sample,
                           const float gyro_bias[3],
                           float gyro_robot[3],
                           float accel_robot[3]);

/**
 * Read the most-recent motor commands from the active motor driver if
 * available. If no driver is active, both outputs will be set to 0.0f.
 */
void getLastMotorCommands(float &left_cmd, float &right_cmd);

/**
 * Receive the latest available sample from the queue, discarding any older stale samples.
 * Returns true if a sample was retrieved, false otherwise.
 */
bool receiveLatestSample(QueueHandle_t queue, IMUSample &sample, uint32_t wait_ticks);

/**
 * Emit raw IMU debug logs (throttled) when `ENABLE_DEBUG_LOGS` is defined.
 * This helper centralizes the throttling behavior used by `imuConsumerTask`.
 * @param sample IMU sample to log
 * @param last_print_ms mutable timestamp used to enforce throttle
 * @param interval_ms throttling interval in milliseconds (default 1000)
 */
void emitImuDebugLogsIfEnabled(const IMUSample &sample, uint32_t &last_print_ms,
                               uint32_t interval_ms = 1000);

} // namespace imu_consumer
} // namespace abbot
