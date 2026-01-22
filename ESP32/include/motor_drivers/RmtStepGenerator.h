#pragma once

#include "IStepGenerator.h"
#include <driver/rmt.h>

namespace abbot {
namespace motor {

/**
 * @brief RMT-based Step Generator for ESP32-S3.
 * Uses the Remote Control (RMT) peripheral to generate precise pulse trains
 * even at very low frequencies, which is often difficult with MCPWM/PWM.
 */
class RmtStepGenerator : public IStepGenerator {
public:
    RmtStepGenerator();
    virtual ~RmtStepGenerator() override;

    /**
     * @brief Initialize RMT channels for two motors.
     * @param left_step_pin GPIO for left motor pulse
     * @param right_step_pin GPIO for right motor pulse
     * @param common_anode If true, idle level is HIGH
     * @return true if successful
     */
    bool init(int left_step_pin, int right_step_pin, bool common_anode = false) override;

    /**
     * @brief Set frequency for a specific motor.
     * @param is_left True for left motor, false for right
     * @param freq_hz Frequency in Hz (0 to stop)
     */
    void setFrequency(bool is_left, uint32_t freq_hz) override;

    /**
     * @brief Stop pulse generation immediately.
     */
    void stopAll() override;

private:
    int m_left_pin = -1;
    int m_right_pin = -1;
    bool m_common_anode = false;
    uint32_t m_current_freq_l = 0;
    uint32_t m_current_freq_r = 0;

    rmt_channel_t m_left_chan = RMT_CHANNEL_2;
    rmt_channel_t m_right_chan = RMT_CHANNEL_3;

    void updateHardware(rmt_channel_t channel, uint32_t freq);
};

} // namespace motor
} // namespace abbot
