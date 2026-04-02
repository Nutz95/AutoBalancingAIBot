#pragma once

#include "IStepGenerator.h"

#if !defined(UNIT_TEST_HOST)
#include <esp32-hal-timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>
#endif

namespace abbot {
namespace motor {

/**
 * @brief Marlin-style software step scheduler using one shared hardware timer.
 *
 * The timer runs only while at least one motor is active, and its cadence is
 * adapted to the fastest requested STEP frequency. The cadence deliberately
 * targets roughly two timer ticks per generated STEP period (high + low), which
 * keeps interrupt pressure low while preserving deterministic pulse generation.
 */
class SoftwareStepGenerator : public IStepGenerator {
public:
    SoftwareStepGenerator();
    virtual ~SoftwareStepGenerator() override;

    bool init(int left_step_pin, int right_step_pin, bool common_anode = false) override;
    bool setFrequency(bool is_left, uint32_t freq_hz) override;
    void stopAll() override;

private:
    static constexpr uint32_t kTimerDivider = 80;          // 1 tick = 1 us
    static constexpr uint32_t kMinTickPeriodUs = 5;        // Upper CPU bound / max pulse capability
    static constexpr uint32_t kMaxTickPeriodUs = 250;      // Strongly reduce ISR load at low speed
    static constexpr uint8_t kPulseWidthTicks = 1;         // 1 timer tick high pulse
    static constexpr uint8_t kLowGapTicks = 1;             // 1 timer tick guaranteed low gap
    static constexpr uint32_t kAccumulatorScale = 1000000UL;

    int m_left_pin = -1;
    int m_right_pin = -1;
    bool m_common_anode = false;
    uint32_t m_current_freq_l = 0;
    uint32_t m_current_freq_r = 0;

#if !defined(UNIT_TEST_HOST)
    struct ChannelState {
        volatile uint32_t target_freq_hz = 0;
        volatile uint32_t increment_per_tick = 0;
        volatile uint32_t accumulator = 0;
        volatile uint8_t pulse_ticks_remaining = 0;
        volatile uint8_t low_gap_ticks_remaining = 0;
    };

    static SoftwareStepGenerator* s_instance;

    ChannelState m_left_state;
    ChannelState m_right_state;
    hw_timer_t* m_timer = nullptr;
    bool m_timer_running = false;
    uint32_t m_tick_period_us = kMaxTickPeriodUs;
    portMUX_TYPE m_mux = portMUX_INITIALIZER_UNLOCKED;

    static void IRAM_ATTR onTimerThunk();
    void IRAM_ATTR onTimer();
    void IRAM_ATTR serviceChannel(ChannelState& state, int pin);
    void updateChannelIncrementsLocked(uint32_t tick_period_us);
    uint32_t computeDesiredTickPeriodUs(uint32_t max_freq_hz) const;
    void IRAM_ATTR writeStepPinLevel(int pin, bool level);
    void writeIdleLevels();
#endif
};

} // namespace motor
} // namespace abbot