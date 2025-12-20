#include "tuning_capture.h"
#include "SystemTasks.h"
#include "logging.h"
#include <Arduino.h>
#include <cmath>
#include <cstdio>

namespace abbot {
namespace tuning {

static uint32_t g_target = 0;
static uint32_t g_count = 0;
static bool g_stream = true;
static bool g_active = false;
static bool g_stats_only = false;

// Welford accumulators for pitch and pitch_rate
static float pitch_mean = 0.0f;
static float pitch_M2 = 0.0f;
static float pr_mean = 0.0f;
static float pr_M2 = 0.0f;
static uint32_t g_last_progress_ms = 0;
static void (*g_on_complete)() = nullptr;

void init() {
  g_active = false;
  g_target = 0;
  g_count = 0;
}

void startCapture(uint32_t nSamples, bool streamCsv, bool statsOnly) {
  g_target = nSamples;
  g_count = 0;
  g_stream = streamCsv;
  g_stats_only = statsOnly;
  g_active = true;
  pitch_mean = pitch_M2 = pr_mean = pr_M2 = 0.0f;
  if (g_stream) {
    // Print CSV header for consumers (should be gated by current mask)
    // New columns: ax,ay,az (m/s^2), gx,gy,gz (rad/s), temp_C
    LOG_PRINTLN(abbot::log::CHANNEL_TUNING,
                "timestamp_ms,pitch_deg,pitch_rad,pitch_rate_deg,pitch_rate_"
                "rad,ax,ay,az,gx,gy,gz,temp_C,left_cmd,right_cmd");
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT,
                "TUNING: capture started (stream)");
  } else if (g_stats_only) {
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT,
                "TUNING: capture started (stats-only)");
  } else {
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT,
                "TUNING: capture started (no-stream)");
  }
  // Request a Madgwick warm-up period to reduce initial transient prior to
  // emitting CSV Default warm-up: ~12 seconds worth of samples (based on fusion
  // sample rate)
  const float DEFAULT_WARMUP_SECONDS = 1.0f;
  // Default warmup reduced to 1s for faster capture start; callers may override
  requestTuningWarmupSeconds(DEFAULT_WARMUP_SECONDS);
}

void setOnCaptureComplete(void (*cb)()) { g_on_complete = cb; }

static void printSummary() {
  char buf[256];
  float pitch_std = (g_count > 1) ? sqrtf(pitch_M2 / (g_count - 1)) : 0.0f;
  float pr_std = (g_count > 1) ? sqrtf(pr_M2 / (g_count - 1)) : 0.0f;
  snprintf(buf, sizeof(buf),
           "TUNING SUMMARY: samples=%u, pitch_mean=%.6f, pitch_std=%.6f, "
           "pr_mean=%.6f, pr_std=%.6f",
           (unsigned)g_count, pitch_mean, pitch_std, pr_mean, pr_std);
  LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, buf);
  // Simple suggested beta heuristic (very conservative)
  float suggested = 0.08f;
  // adjust slightly based on pitch_rate noise
  suggested = 0.02f + fminf(0.3f, pr_std * 0.1f + 0.05f);
  snprintf(buf, sizeof(buf), "Suggested beta ~ %.3f", suggested);
  LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, buf);
}

void stopCapture() {
  if (!g_active) {
    return;
  }
  g_active = false;
  printSummary();
  LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "TUNING: capture stopped (auto)");
  // Try to restore previous logging mask if the caller pushed one.
  if (!abbot::log::popChannelMask()) {
    // Do not crash; just log an error so the operator can notice mismatched
    // push/pop
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT,
                "TUNING: warning - popChannelMask() failed (no matching push)");
  }
  g_stats_only = false;
  // Invoke completion callback if registered
  if (g_on_complete) {
    g_on_complete();
  }
}

void submitSample(uint32_t ts_ms, float pitch_deg, float /*pitch_rad*/,
                  float pitch_rate_deg, float /*pitch_rate_rad*/, float ax,
                  float ay, float az, float gx, float gy, float gz,
                  float temp_C, float left_cmd, float right_cmd) {
  if (!g_active) {
    return;
  }

  // Emit CSV if requested (we emit on CHANNEL_TUNING; caller should have
  // enabled mask)
  if (g_stream && abbot::log::isChannelEnabled(abbot::log::CHANNEL_TUNING)) {
    LOG_PRINTF(
        abbot::log::CHANNEL_TUNING,
        "%u,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.3f,%.6f,%.6f\n",
        ts_ms, pitch_deg, pitch_deg * 3.14159265f / 180.0f, pitch_rate_deg,
        pitch_rate_deg * 3.14159265f / 180.0f, ax, ay, az, gx, gy, gz, temp_C,
        left_cmd, right_cmd);
  }

  // Welford update for pitch
  g_count++;
  float delta = pitch_deg - pitch_mean;
  pitch_mean += delta / g_count;
  pitch_M2 += delta * (pitch_deg - pitch_mean);

  // Welford update for pitch_rate
  float delta2 = pitch_rate_deg - pr_mean;
  pr_mean += delta2 / g_count;
  pr_M2 += delta2 * (pitch_rate_deg - pr_mean);

  // Periodic progress updates when in stats-only mode
  if (g_stats_only) {
    uint32_t now = millis();
    if (g_last_progress_ms == 0) {
      g_last_progress_ms = now;
    }
    if ((uint32_t)(now - g_last_progress_ms) >= 2000) {
      g_last_progress_ms = now;
      // compute current std dev (sample std)
      float pitch_std = (g_count > 1) ? sqrtf(pitch_M2 / (g_count - 1)) : 0.0f;
      float pr_std = (g_count > 1) ? sqrtf(pr_M2 / (g_count - 1)) : 0.0f;
      // progress percent and simple ascii bar
      int pct = 0;
      int barWidth = 20;
      int filled = 0;
      if (g_target > 0) {
        pct = (int)((g_count * 100) / g_target);
        filled = (int)(((uint64_t)g_count * (uint64_t)barWidth) / g_target);
        if (filled > barWidth) {
          filled = barWidth;
        }
      }
      char bar[32];
      for (int i = 0; i < barWidth; ++i) {
        bar[i] = (i < filled) ? '#' : '-';
      }
      bar[barWidth] = '\0';
      char msg[256];
      snprintf(msg, sizeof(msg),
               "TUNING PROGRESS: [%s] %d%% (%u/%u) pitch_mean=%.6f "
               "pitch_std=%.6f pr_mean=%.6f pr_std=%.6f",
               bar, pct, (unsigned)g_count, (unsigned)g_target, pitch_mean,
               pitch_std, pr_mean, pr_std);
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, msg);
    }
  }

  if (g_target > 0 && g_count >= g_target) {
    // finish
    stopCapture();
  }
}

bool isCapturing() { return g_active; }

} // namespace tuning
} // namespace abbot
