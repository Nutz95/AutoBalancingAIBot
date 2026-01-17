#pragma once

#include <stdint.h>
#include <string>

namespace abbot {
namespace balancing {

/**
 * @brief Identifiers for available balancing strategies.
 */
enum class StrategyType {
    LEGACY_PID = 0,
    CASCADED_LQR = 1
};

/**
 * @brief Interface for a balancing control strategy.
 * 
 * Each strategy defines how it computes the motor command from IMU and encoder data,
 * and how it manages its own configuration and telemetry.
 */
class IBalancingStrategy {
public:
    virtual ~IBalancingStrategy() = default;

    /**
     * @brief Unique name of the strategy.
     */
    virtual const char* getName() const = 0;

    /**
     * @brief Strategy type identifier.
     */
    virtual StrategyType getType() const = 0;

    /**
     * @brief Initialize the strategy (load config, etc.).
     */
    virtual void init() = 0;

    /**
     * @brief Reset runtime state (integrators, filters) before starting.
     * @param initial_pitch_rad Starting pitch to avoid jump.
     */
    virtual void reset(float initial_pitch_rad) = 0;

    /**
     * @brief Result of the compute call includes telemetry data for logging.
     */
    struct Result {
        float command;
        float steer;
        float iterm; // Or adaptive trim equivalent
        float lqr_angle;
        float lqr_gyro;
        float lqr_dist;
        float lqr_speed;
    };

    /**
     * @brief Compute the balancing cycle.
     * 
     * @param pitch_rad Current fused pitch (radians).
     * @param pitch_rate_rads Current fused pitch rate (rad/s).
     * @param yaw_rate_rads Current yaw rate from gyro Z (rad/s).
     * @param dt_s Cycle time in seconds.
     * @param enc_l_ticks Left encoder cumulative ticks.
     * @param enc_r_ticks Right encoder cumulative ticks.
     * @param v_enc_ticks_s Average wheel velocity in ticks/s.
     * @return Result Command and telemetry.
     */
    virtual Result compute(float pitch_rad, float pitch_rate_rads, float yaw_rate_rads, float dt_s,
                         int32_t enc_l_ticks, int32_t enc_r_ticks,
                         float v_enc_ticks_s) = 0;

    /**
     * @brief Handle remote drive setpoints.
     */
    virtual void setDriveSetpoints(float v_norm, float w_norm) = 0;

    /**
     * @brief Persistence: Load parameters from NVS.
     */
    virtual void loadConfig() = 0;

    /**
     * @brief Persistence: Save current parameters to NVS.
     */
    virtual void saveConfig() = 0;

    /**
     * @brief Reset parameters to factory defaults.
     */
    virtual void resetToDefaults() = 0;
};

} // namespace balancing
} // namespace abbot
