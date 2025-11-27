// tuning_capture.h
#pragma once

#include <cstdint>

namespace abbot {
namespace tuning {

// Initialize tuning capture module (no dynamic allocation)
void init();

// Start a capture for N samples.
// If streamCsv==true, CSV rows will be emitted on the TUNING channel while
// collecting. If statsOnly==true, no CSV is emitted but periodic (2s)
// progress + current stats are printed until capture finishes.
void startCapture(uint32_t nSamples, bool streamCsv=true, bool statsOnly=false);

// Stop an ongoing capture (if any). This will finalize stats and print summary.
void stopCapture();

// Called by SystemTasks or IMU loop to submit the latest sample (pitch_deg, pitch_rate_deg, left_cmd, right_cmd, ts_ms)
void submitSample(uint32_t ts_ms,
				  float pitch_deg,
				  float pitch_rad,
				  float pitch_rate_deg,
				  float pitch_rate_rad,
				  float ax, float ay, float az,
				  float gx, float gy, float gz,
				  float temp_C,
				  float left_cmd, float right_cmd);

// Query whether a capture is active
bool isCapturing();

// Register a callback invoked when a capture finishes (called from stopCapture()).
// The callback runs in the context of the caller of stopCapture (IMU task), so it
// should be short and non-blocking.
void setOnCaptureComplete(void (*cb)());

} // namespace tuning
} // namespace abbot
