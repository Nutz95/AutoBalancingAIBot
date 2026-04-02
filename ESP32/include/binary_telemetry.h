#pragma once
#include <cstdint>

namespace abbot {
namespace telemetry {

#pragma pack(push, 1)
/**
 * @brief Binary packet for high-speed UDP telemetry.
 * Size: 180 bytes.
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
    uint32_t last_encoder_age_ms;
    uint32_t bus_latency_us;
    uint32_t ack_pending_left_us;
    uint32_t ack_pending_right_us;
    uint32_t bus_latency_left_us;
    uint32_t bus_latency_right_us;
    uint32_t bus_latency_left_age_ms;
    uint32_t bus_latency_right_age_ms;
    
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

    // Command saturation / motor mixing diagnostics
    float cmd_raw;
    float cmd_final;
    float left_preclip;
    float right_preclip;
    float left_postclip;
    float right_postclip;
    uint32_t sat_flags; // bit0=cmd clipped, bit1=left clipped, bit2=right clipped

    // Motor speed feedback from RS485 (RPM, signed: positive=forward)
    float motor_rpm_l;
    float motor_rpm_r;

    // Step/Dir diagnostic path: actual applied STEP frequency.
    uint32_t step_hz_l;
    uint32_t step_hz_r;
};
#pragma pack(pop)

} // namespace telemetry
} // namespace abbot
