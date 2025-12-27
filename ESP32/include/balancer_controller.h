#pragma once

#include <stdint.h>

namespace abbot {
namespace balancer {
// High-level balancer control API. These functions are thin wrappers that
// provide a centralized place for balancer-related operations (start/stop,
// gain/deadband management, calibration). They currently forward to the
// implementations in `SystemTasks.cpp` but allow future refactor into this
// module without changing callers.
namespace controller {
void init();
// Start the controller with the current fused pitch (radians).
// Passing the current fused pitch lets callers provide the value
// (e.g. from `abbot::getFusedPitch()`) so this module does not
// depend on `SystemTasks.h`.
void start(float fused_pitch_rad);
void stop();
bool isActive();
void setGains(float kp, float ki, float kd);
void getGains(float &kp, float &ki, float &kd);
void resetGainsToDefaults(); // Reset gains to compile-time defaults from
                             // balancer_config.h
void setDeadband(float db);
float getDeadband();
void calibrateDeadband();
void setMinCmd(float min_cmd);
float getMinCmd();
// Called each IMU loop to compute (and optionally apply) motor command.
// Arguments: fused_pitch_rad (radians), fused_pitch_rate_rads (radians/sec),
// dt_s (seconds). Returns computed normalized command in [-1,1]. Note: Pitch
// sign is handled by axis mapping in FusionConfig (accel_sign/gyro_sign).
float processCycle(float fused_pitch_rad, float fused_pitch_rate_rads,
                   float dt_s);

// High-level drive interface: set desired forward (v) and turn (w) commands
// v, w are normalized in [-1, 1]. This interface is intentionally
// minimal: the controller will convert v -> pitch setpoint and apply
// slew limiting internally. w (turn) may be ignored depending on
// platform/configuration.
void setDriveSetpoints(float v_norm, float w_norm);
// Autotuning API
void startAutotune();
void stopAutotune();
bool isAutotuning();
const char *getAutotuneStatus();
void applyAutotuneGains(); // Apply gains if autotune completed successfully

// Autotune configuration setters
void setAutotuneRelay(float amplitude);
void setAutotuneDeadband(float deadband_deg);
void setAutotuneMaxAngle(float max_pitch_deg);

// Motor gain adjustment (for asymmetric motor compensation)
void setMotorGains(float left_gain, float right_gain);
void getMotorGains(float &left_gain, float &right_gain);

// Calibrated trim management: allows storing a fixed reference trim
// (captured when robot is on a support at true vertical) instead of
// dynamically capturing at each START.
void calibrateTrim();   // Capture current pitch as calibrated trim, persist to NVS
void showTrim();        // Display calibrated and dynamic trim values
void resetTrim();       // Clear calibrated trim (reverts to dynamic capture)

// Provide latest IMU sample (robot frame) for logging/diagnostics.
// accel in m/s^2, gyro in rad/s.
void setLatestImuSample(const float accel_robot[3], const float gyro_robot[3]);
} // namespace controller

} // namespace balancer
} // namespace abbot
