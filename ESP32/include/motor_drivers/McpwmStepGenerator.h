#pragma once

#include <cstdint>

namespace abbot {
namespace motor {

/**
 * @brief MCPWM-based Step Generator for ESP32-S3.
 * Provides high-precision pulse generation with low CPU overhead.
 */
class McpwmStepGenerator {
public:
    McpwmStepGenerator();
    ~McpwmStepGenerator();

    /**
     * @brief Initialize MCPWM for two motors.
     * @param left_step_pin GPIO for left motor pulse
     * @param right_step_pin GPIO for right motor pulse
     * @param common_anode If true, duty cycle is inverted for sinking logic
     * @return true if successful
     */
    bool init(int left_step_pin, int right_step_pin, bool common_anode = false);

    /**
     * @brief Set frequency for a specific motor.
     * @param is_left True for left motor, false for right
     * @param freq_hz Frequency in Hz (0 to stop)
     */
    void setFrequency(bool is_left, uint32_t freq_hz);

    /**
     * @brief Stop pulse generation immediately.
     */
    void stopAll();

private:
    int m_left_pin = -1;
    int m_right_pin = -1;
    bool m_common_anode = false;
    uint32_t m_current_freq_l = 0;
    uint32_t m_current_freq_r = 0;

    void updateHardware(int timer_idx, uint32_t freq);
};

} // namespace motor
} // namespace abbot
