#pragma once
#include <cstdint>

namespace abbot {
namespace telemetry {

#pragma pack(push, 1)
/**
 * @brief Binary packet for high-speed UDP telemetry.
 * Size: 108 bytes (27 fields * 4 bytes)
 */
struct TelemetryPacket {
    uint32_t magic = 0xABBA0001; // Magic header for packet validation
    uint32_t timestamp_ms;
    
    // Control State
    float pitch_deg;
    float pid_in_deg;
    float pid_out;
    float iterm;
    float cmd;
    float steer;
    
    // Sensors
    float ax, ay, az;
    float gx, gy, gz;
    float loop_freq_hz;
    
    // Motors/Odometry
    int32_t enc_l, enc_r;
    uint32_t bus_latency_us;
    uint32_t ack_pending_time_us;
    
    // LQR/PID Decomposition
    float lqr_angle;
    float lqr_gyro;
    float lqr_dist;
    float lqr_speed;
    
    // CPU Load
    float cpu0_pct;
    float cpu1_pct;
    
    // Profiling (microseconds)
    uint32_t prof_f; // Fusion
    uint32_t prof_l; // LQR/Control
    uint32_t prof_t; // Total
    uint32_t prof_log; // Logging overhead
};
#pragma pack(pop)

} // namespace telemetry
} // namespace abbot
