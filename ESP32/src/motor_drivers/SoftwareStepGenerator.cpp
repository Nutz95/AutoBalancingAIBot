#include "motor_drivers/SoftwareStepGenerator.h"
#include "../../config/motor_configs/mks_servo_config.h"
#include <Arduino.h>

#if !defined(UNIT_TEST_HOST)
#include <soc/gpio_struct.h>

namespace abbot {
namespace motor {

SoftwareStepGenerator* SoftwareStepGenerator::s_instance = nullptr;

SoftwareStepGenerator::SoftwareStepGenerator() {}

SoftwareStepGenerator::~SoftwareStepGenerator() {
    stopAll();
    if (m_timer) {
        timerAlarmDisable(m_timer);
        timerDetachInterrupt(m_timer);
        timerEnd(m_timer);
        m_timer = nullptr;
    }
    s_instance = nullptr;
}

bool SoftwareStepGenerator::init(int left_step_pin, int right_step_pin, bool common_anode) {
    m_left_pin = left_step_pin;
    m_right_pin = right_step_pin;
    m_common_anode = common_anode;
    m_current_freq_l = 0;
    m_current_freq_r = 0;
    m_tick_period_us = kMaxTickPeriodUs;

    portENTER_CRITICAL(&m_mux);
    m_left_state = ChannelState{};
    m_right_state = ChannelState{};
    portEXIT_CRITICAL(&m_mux);

    writeIdleLevels();

    if (m_timer) {
        timerAlarmDisable(m_timer);
        timerDetachInterrupt(m_timer);
        timerEnd(m_timer);
        m_timer = nullptr;
    }
    m_timer_running = false;

    s_instance = this;
    m_timer = timerBegin(0, (uint16_t)kTimerDivider, true);
    if (!m_timer) {
        s_instance = nullptr;
        return false;
    }

    timerAttachInterrupt(m_timer, &SoftwareStepGenerator::onTimerThunk, true);
    timerAlarmWrite(m_timer, m_tick_period_us, true);
    return true;
}

bool SoftwareStepGenerator::setFrequency(bool is_left, uint32_t freq_hz) {
    uint32_t max_freq = MKS_SERVO_STEP_MAX_HZ;
    const uint32_t software_limit = 1000000UL / (uint32_t)(2U * kMinTickPeriodUs);
    if (max_freq > software_limit) {
        max_freq = software_limit;
    }
    if (freq_hz > max_freq) {
        freq_hz = max_freq;
    }

    uint32_t desired_tick_us = kMaxTickPeriodUs;
    bool should_run = false;
    bool tick_changed = false;
    portENTER_CRITICAL(&m_mux);
    ChannelState& state = is_left ? m_left_state : m_right_state;
    state.target_freq_hz = freq_hz;
    if (freq_hz == 0U) {
        state.increment_per_tick = 0;
        state.accumulator = 0;
        state.pulse_ticks_remaining = 0;
        state.low_gap_ticks_remaining = 0;
        writeStepPinLevel(is_left ? m_left_pin : m_right_pin, m_common_anode ? HIGH : LOW);
    }
    if (is_left) {
        m_current_freq_l = freq_hz;
    } else {
        m_current_freq_r = freq_hz;
    }
    const uint32_t max_active_freq = (m_left_state.target_freq_hz > m_right_state.target_freq_hz)
                                         ? m_left_state.target_freq_hz
                                         : m_right_state.target_freq_hz;
    should_run = (max_active_freq != 0U);
    desired_tick_us = computeDesiredTickPeriodUs(max_active_freq);
    tick_changed = (desired_tick_us != m_tick_period_us);
    m_tick_period_us = desired_tick_us;
    updateChannelIncrementsLocked(desired_tick_us);
    portEXIT_CRITICAL(&m_mux);

    if (m_timer) {
        if (should_run) {
            if (!m_timer_running || tick_changed) {
                timerAlarmDisable(m_timer);
                timerWrite(m_timer, 0);
                timerAlarmWrite(m_timer, desired_tick_us, true);
                timerAlarmEnable(m_timer);
                m_timer_running = true;
            }
        } else if (m_timer_running) {
            timerAlarmDisable(m_timer);
            m_timer_running = false;
            writeIdleLevels();
        }
    }
    return true;
}

void SoftwareStepGenerator::stopAll() {
    setFrequency(true, 0);
    setFrequency(false, 0);
}

void IRAM_ATTR SoftwareStepGenerator::onTimerThunk() {
    if (s_instance) {
        s_instance->onTimer();
    }
}

void IRAM_ATTR SoftwareStepGenerator::onTimer() {
    portENTER_CRITICAL_ISR(&m_mux);
    serviceChannel(m_left_state, m_left_pin);
    serviceChannel(m_right_state, m_right_pin);
    portEXIT_CRITICAL_ISR(&m_mux);
}

void IRAM_ATTR SoftwareStepGenerator::serviceChannel(ChannelState& state, int pin) {
    const bool idle_level = m_common_anode ? HIGH : LOW;
    const bool active_level = !idle_level;

    if (state.pulse_ticks_remaining > 0) {
        state.pulse_ticks_remaining--;
        if (state.pulse_ticks_remaining == 0) {
            writeStepPinLevel(pin, idle_level);
            state.low_gap_ticks_remaining = kLowGapTicks;
        }
    } else if (state.low_gap_ticks_remaining > 0) {
        state.low_gap_ticks_remaining--;
    }

    const uint32_t freq_hz = state.target_freq_hz;
    if (freq_hz == 0U) {
        state.accumulator = 0;
        return;
    }

    state.accumulator += state.increment_per_tick;
    if (state.pulse_ticks_remaining == 0 &&
        state.low_gap_ticks_remaining == 0 &&
        state.accumulator >= kAccumulatorScale) {
        state.accumulator -= kAccumulatorScale;
        writeStepPinLevel(pin, active_level);
        state.pulse_ticks_remaining = kPulseWidthTicks;
    }
}

void SoftwareStepGenerator::updateChannelIncrementsLocked(uint32_t tick_period_us) {
    m_left_state.increment_per_tick = m_left_state.target_freq_hz * tick_period_us;
    m_right_state.increment_per_tick = m_right_state.target_freq_hz * tick_period_us;
}

uint32_t SoftwareStepGenerator::computeDesiredTickPeriodUs(uint32_t max_freq_hz) const {
    if (max_freq_hz == 0U) {
        return kMaxTickPeriodUs;
    }

    uint32_t period_us = 1000000UL / max_freq_hz;
    uint32_t desired = period_us / 2U;
    if (desired == 0U) {
        desired = 1U;
    }
    if (desired < kMinTickPeriodUs) {
        desired = kMinTickPeriodUs;
    }
    if (desired > kMaxTickPeriodUs) {
        desired = kMaxTickPeriodUs;
    }
    return desired;
}

void IRAM_ATTR SoftwareStepGenerator::writeStepPinLevel(int pin, bool level) {
    if (pin < 0) {
        return;
    }

    if (pin < 32) {
        const uint32_t mask = (1UL << pin);
        if (level) {
            GPIO.out_w1ts = mask;
        } else {
            GPIO.out_w1tc = mask;
        }
        return;
    }

    const uint32_t mask = (1UL << (pin - 32));
    if (level) {
        GPIO.out1_w1ts.val = mask;
    } else {
        GPIO.out1_w1tc.val = mask;
    }
}

void SoftwareStepGenerator::writeIdleLevels() {
    const bool idle_level = m_common_anode ? HIGH : LOW;
    digitalWrite(m_left_pin, idle_level);
    digitalWrite(m_right_pin, idle_level);
}

} // namespace motor
} // namespace abbot

#else

namespace abbot {
namespace motor {

SoftwareStepGenerator::SoftwareStepGenerator() {}
SoftwareStepGenerator::~SoftwareStepGenerator() {}

bool SoftwareStepGenerator::init(int left_step_pin, int right_step_pin, bool common_anode) {
    (void)left_step_pin;
    (void)right_step_pin;
    (void)common_anode;
    return true;
}

bool SoftwareStepGenerator::setFrequency(bool is_left, uint32_t freq_hz) {
    (void)is_left;
    (void)freq_hz;
    return true;
}

void SoftwareStepGenerator::stopAll() {}

} // namespace motor
} // namespace abbot

#endif