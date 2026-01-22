#pragma once

#include <cstdint>

namespace abbot {
namespace motor {

/**
 * @brief Common interface for Step pulse generators (MCPWM, RMT, LEDC, etc.)
 */
class IStepGenerator {
public:
    virtual ~IStepGenerator() = default;

    /**
     * @brief Initialize hardware for two motors.
     */
    virtual bool init(int left_step_pin, int right_step_pin, bool common_anode = false) = 0;

    /**
     * @brief Set frequency for a specific motor channel.
     * @param is_left True for left motor, false for right
     * @param freq_hz Frequency in Hz (0 to stop)
     */
    virtual void setFrequency(bool is_left, uint32_t freq_hz) = 0;

    /**
     * @brief Stop all pulse generation immediately.
     */
    virtual void stopAll() = 0;
};

} // namespace motor
} // namespace abbot
