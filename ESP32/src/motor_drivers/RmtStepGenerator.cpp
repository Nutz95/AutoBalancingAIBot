#include "motor_drivers/RmtStepGenerator.h"
#include "../../config/motor_configs/mks_servo_config.h"
#include <Arduino.h>

namespace abbot {
namespace motor {

RmtStepGenerator::RmtStepGenerator() {}
RmtStepGenerator::~RmtStepGenerator() {
    stopAll();
#if !defined(UNIT_TEST_HOST)
    if (m_left_mutex)  { vSemaphoreDelete(m_left_mutex);  m_left_mutex  = nullptr; }
    if (m_right_mutex) { vSemaphoreDelete(m_right_mutex); m_right_mutex = nullptr; }
#endif
}

bool RmtStepGenerator::init(int left_step_pin, int right_step_pin, bool common_anode) {
    m_left_pin = left_step_pin;
    m_right_pin = right_step_pin;
    m_common_anode = common_anode;

#if !defined(UNIT_TEST_HOST)
    if (!m_left_mutex) {
        m_left_mutex = xSemaphoreCreateMutex();
        if (!m_left_mutex) return false;
    }
    if (!m_right_mutex) {
        m_right_mutex = xSemaphoreCreateMutex();
        if (!m_right_mutex) return false;
    }
#endif

    rmt_config_t config_l = {};
    config_l.rmt_mode = RMT_MODE_TX;
    config_l.channel = m_left_chan;
    config_l.gpio_num = (gpio_num_t)m_left_pin;
    config_l.clk_div = 160; // 0.5MHz clock (80MHz / 160) -> 2us per tick
    config_l.mem_block_num = 1;
    config_l.tx_config.loop_en = true;
    config_l.tx_config.carrier_en = false;
    config_l.tx_config.idle_output_en = true;
    config_l.tx_config.idle_level = m_common_anode ? RMT_IDLE_LEVEL_HIGH : RMT_IDLE_LEVEL_LOW;

    rmt_config_t config_r = config_l;
    config_r.channel = m_right_chan;
    config_r.gpio_num = (gpio_num_t)m_right_pin;

    esp_err_t err;
    err = rmt_config(&config_l);
    if (err != ESP_OK) return false;
    err = rmt_driver_install(config_l.channel, 0, 0);
    if (err != ESP_OK) return false;
    
    err = rmt_config(&config_r);
    if (err != ESP_OK) return false;
    err = rmt_driver_install(config_r.channel, 0, 0);
    if (err != ESP_OK) return false;

    m_current_freq_l = 0;
    m_current_freq_r = 0;
    return true;
}

bool RmtStepGenerator::setFrequency(bool is_left, uint32_t freq_hz) {
#if !defined(UNIT_TEST_HOST)
    SemaphoreHandle_t chan_mutex = is_left ? m_left_mutex : m_right_mutex;
    if (chan_mutex && xSemaphoreTake(chan_mutex, pdMS_TO_TICKS(2)) != pdTRUE) {
        return false;
    }
#endif

    uint32_t& current_freq = is_left ? m_current_freq_l : m_current_freq_r;
    rmt_channel_t chan = is_left ? m_left_chan : m_right_chan;

    if (freq_hz > MKS_SERVO_STEP_MAX_HZ) {
        freq_hz = MKS_SERVO_STEP_MAX_HZ;
    }

    // Stop if frequency is too low or zero
    if (freq_hz < 8) {
        if (current_freq != 0) {
            rmt_tx_stop(chan);
            current_freq = 0;
        }
#if !defined(UNIT_TEST_HOST)
        if (chan_mutex) xSemaphoreGive(chan_mutex);
#endif
        return true;
    }

    if (current_freq != freq_hz) {
        updateHardware(chan, freq_hz);
        current_freq = freq_hz;
    }

#if !defined(UNIT_TEST_HOST)
    if (chan_mutex) xSemaphoreGive(chan_mutex);
#endif
    return true;
}

void RmtStepGenerator::updateHardware(rmt_channel_t channel, uint32_t freq) {
    // 1. Force stop the channel to allow writing new items (mandatory for loop mode)
    rmt_tx_stop(channel);

    // 2. Calculate ticks for 0.5MHz clock (2us per tick)
    // 1s = 500,000 ticks
    // period (ticks) = 500,000 / freq
    // half_period (ticks) = 250,000 / freq
    uint32_t half_period_ticks = 250000 / freq;

    // RMT duration field is 15 bits (0..32767)
    if (half_period_ticks > 32767) {
        half_period_ticks = 32767;
    }
    if (half_period_ticks == 0) {
        half_period_ticks = 1;
    }

    // 3. Define the pulse item [Phase 1] [Phase 2]
    // Each item represents TWO transitions (high and low).
    // duration0 (half_period), level0 (high), duration1 (half_period), level1 (low)
    uint32_t level1 = m_common_anode ? 0 : 1;
    uint32_t level2 = m_common_anode ? 1 : 0;

    rmt_item32_t items[1] = {
        {{{ (uint16_t)half_period_ticks, level1, (uint16_t)half_period_ticks, level2 }}}
    };

    // 4. Write directly into RMT memory (non-blocking) and restart.
    // rmt_fill_tx_items is a direct memcpy with no internal semaphore,
    // avoiding the potential blocking found in rmt_write_items on ESP32-S3
    // when loop_en=true (tx_sem may never be signalled in that mode).
    rmt_fill_tx_items(channel, items, 1, 0);
    rmt_tx_start(channel, false);
}

void RmtStepGenerator::stopAll() {
    setFrequency(true, 0);
    setFrequency(false, 0);
}

} // namespace motor
} // namespace abbot
