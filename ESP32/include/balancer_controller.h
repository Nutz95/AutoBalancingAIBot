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
    void setDeadband(float db);
    float getDeadband();
    void calibrateDeadband();
    // Called each IMU loop to compute (and optionally apply) motor command.
    // Arguments: fused_pitch_rad (radians), fused_pitch_rate_rads (radians/sec), dt_s (seconds).
    // Returns computed normalized command in [-1,1].
    float processCycle(float fused_pitch_rad, float fused_pitch_rate_rads, float dt_s);
}

} // namespace balancer
} // namespace abbot
