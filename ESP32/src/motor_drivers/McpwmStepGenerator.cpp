#include "motor_drivers/McpwmStepGenerator.h"
#include "../../config/motor_configs/mks_servo_config.h"
#include <Arduino.h>

#if !defined(UNIT_TEST)
#include "driver/mcpwm.h"

namespace abbot {
namespace motor {

McpwmStepGenerator::McpwmStepGenerator() {}
McpwmStepGenerator::~McpwmStepGenerator() { stopAll(); }

bool McpwmStepGenerator::init(int left_step_pin, int right_step_pin, bool common_anode) {
    m_left_pin = left_step_pin;
    m_right_pin = right_step_pin;
    m_common_anode = common_anode;

    // 1. Initial configuration of GPIOs for MCPWM Unit 0
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, (gpio_num_t)m_left_pin);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, (gpio_num_t)m_right_pin);

    mcpwm_config_t pwm_config;
    pwm_config.frequency = 1000;      // Freq par défaut (sera changée)
    pwm_config.cmpr_a = 0.0;          // 0% duty = Arrêté
    pwm_config.cmpr_b = 0.0;
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    // Pour Common Anode (HIGH=OFF), on utilise DUTY_MODE_1 (Active LOW)
    // Ainsi, Duty 0% => Signal toujours HIGH (OFF). Duty 50% => Pulses 50% LOW (ON).
    pwm_config.duty_mode = m_common_anode ? MCPWM_DUTY_MODE_1 : MCPWM_DUTY_MODE_0;

    // 2. Initialize Timers (unit 0 timer 0 et 1)
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config);

    m_current_freq_l = 0;
    m_current_freq_r = 0;
    return true;
}

void McpwmStepGenerator::setFrequency(bool is_left, uint32_t freq_hz) {
#if !defined(UNIT_TEST)
    mcpwm_timer_t timer = is_left ? MCPWM_TIMER_0 : MCPWM_TIMER_1;
    uint32_t& current_freq = is_left ? m_current_freq_l : m_current_freq_r;

    // Hard safety limit from config
    if (freq_hz > MKS_SERVO_STEP_MAX_HZ) {
        freq_hz = MKS_SERVO_STEP_MAX_HZ;
    }

    if (freq_hz < 10) { // Stop
        if (current_freq != 0) {
            mcpwm_set_duty(MCPWM_UNIT_0, timer, MCPWM_OPR_A, 0.0);
            current_freq = 0;
        }
        return;
    }

    if (current_freq == 0) {
        // Redémarrage : on restaure le duty à 50%
        mcpwm_set_duty(MCPWM_UNIT_0, timer, MCPWM_OPR_A, 50.0);
    }

    if (current_freq != freq_hz) {
        mcpwm_set_frequency(MCPWM_UNIT_0, timer, freq_hz);
        current_freq = freq_hz;
    }
#endif
}

void McpwmStepGenerator::stopAll() {
    setFrequency(true, 0);
    setFrequency(false, 0);
}

} // namespace motor
} // namespace abbot
#endif
