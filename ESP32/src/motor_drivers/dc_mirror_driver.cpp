// dc_mirror_driver.cpp
#include "../include/motor_drivers/dc_mirror_driver.h"
#include "../config/motor_configs/dc_motor_config.h"
#include "../config/motor_configs/motor_common_config.h"
#include "../include/motor_drivers/driver_manager.h"
#include "driver/pcnt.h"
#include "logging.h"
#include <Arduino.h>
#include <esp_timer.h>

// Encoder synchronization primitive (used by legacy ISR helpers)
static portMUX_TYPE s_encoder_mux = portMUX_INITIALIZER_UNLOCKED;

// File-local helpers: setup a PCNT unit and read+accumulate its value.
// PCNT event handling support
static volatile uint8_t s_pcnt_event_flags[4] = {0};
static bool s_pcnt_isr_installed = false;

static void IRAM_ATTR pcnt_event_isr(void *arg) {
  int unit = (int)(uintptr_t)arg;
  if (unit >= 0 && unit < (int)(sizeof(s_pcnt_event_flags) /
                                sizeof(s_pcnt_event_flags[0]))) {
    s_pcnt_event_flags[unit] = 1;
    // clear hardware counter to avoid stuck-at-limit; minimal work in ISR
    pcnt_counter_clear((pcnt_unit_t)unit);
  }
}

static void setup_pcnt_unit(int &out_unit, bool &out_configured,
                            pcnt_unit_t unitId, int pulse_pin, int ctrl_pin) {
  pcnt_config_t cfg;
  memset(&cfg, 0, sizeof(cfg));
  cfg.pulse_gpio_num = pulse_pin;
  cfg.ctrl_gpio_num = ctrl_pin;
  cfg.channel = PCNT_CHANNEL_0;
  cfg.unit = unitId;
  cfg.pos_mode = PCNT_COUNT_INC;
  cfg.neg_mode = PCNT_COUNT_DEC;
  cfg.lctrl_mode = PCNT_MODE_KEEP;
  cfg.hctrl_mode = PCNT_MODE_REVERSE;
  cfg.counter_h_lim = 32767;
  cfg.counter_l_lim = -32768;
  pcnt_unit_config(&cfg);
  pcnt_counter_pause(unitId);
  pcnt_counter_clear(unitId);
  pcnt_filter_disable(unitId);
  pcnt_counter_resume(unitId);
  out_unit = (int)unitId;
  out_configured = true;
  LOG_PRINTF(abbot::log::CHANNEL_MOTOR,
             "DCMirrorDriver: configured PCNT unit=%d pulse=%d ctrl=%d\n",
             (int)unitId, pulse_pin, ctrl_pin);
  // enable high/low limit events and register ISR handler once
  pcnt_event_enable(unitId, PCNT_EVT_H_LIM);
  pcnt_event_enable(unitId, PCNT_EVT_L_LIM);
  if (!s_pcnt_isr_installed) {
    pcnt_isr_service_install(0);
    s_pcnt_isr_installed = true;
  }
  pcnt_isr_handler_add(unitId, pcnt_event_isr, (void *)(uintptr_t)unitId);
}

static void readAndAccumulatePCNT(pcnt_unit_t unitId, int64_t &accum,
                                  int signalsPerPin) {
  int16_t cnt = 0;
  pcnt_get_counter_value(unitId, &cnt);
  int u = (int)unitId;
  accum += (int64_t)cnt * (int64_t)signalsPerPin;
  // Debug log: show when hardware counter contributed
  // PCNT logging disabled to avoid console pollution during latency measurements
  // Clear event flag if set
  if (u >= 0 &&
      u < (int)(sizeof(s_pcnt_event_flags) / sizeof(s_pcnt_event_flags[0]))) {
    s_pcnt_event_flags[u] = 0;
  }
  pcnt_counter_clear(unitId);
}

// No external ISRs when using PCNT hardware counters

namespace abbot {
namespace motor {

DCMirrorDriver::DCMirrorDriver()
    : m_enabled(false), m_last_left_cmd(0.0f), m_last_right_cmd(0.0f),
      m_last_left_dir(0), m_last_right_dir(0), m_left_encoder(0),
      m_right_encoder(0), m_left_invert_override_enabled(false),
      m_left_invert_override_value(false),
      m_right_invert_override_enabled(false),
      m_right_invert_override_value(false) {}

// IIR alpha used for speed filtering; named constant for clarity and tuning.
static constexpr float SPEED_IIR_ALPHA = 0.25f;

void DCMirrorDriver::initMotorDriver() {
  // Initialize state
  m_enabled = false;
  m_left_encoder = 0;
  m_right_encoder = 0;
  m_last_left_cmd = 0.0f;
  m_last_right_cmd = 0.0f;
  m_last_left_dir = 0;
  m_last_right_dir = 0;

  // Copy pin defines into members
  m_left_pwm_r_pin = DC_LEFT_PWM_R_PIN;
  m_left_pwm_l_pin = DC_LEFT_PWM_L_PIN;
  m_left_en_r_pin = DC_LEFT_EN_R_PIN;
  m_left_en_l_pin = DC_LEFT_EN_L_PIN;

  m_right_pwm_r_pin = DC_RIGHT_PWM_R_PIN;
  m_right_pwm_l_pin = DC_RIGHT_PWM_L_PIN;
  m_right_en_r_pin = DC_RIGHT_EN_R_PIN;
  m_right_en_l_pin = DC_RIGHT_EN_L_PIN;

  m_pwm_resolution_bits = DC_PWM_RESOLUTION_BITS;
  m_pwm_max_duty = (1 << m_pwm_resolution_bits) - 1;

  // Choose fixed LEDC channels for each pin (or -1 when pin not used)
  m_left_pwm_r_chan = (m_left_pwm_r_pin >= 0) ? 0 : -1;
  m_left_pwm_l_chan = (m_left_pwm_l_pin >= 0) ? 1 : -1;
  m_right_pwm_r_chan = (m_right_pwm_r_pin >= 0) ? 2 : -1;
  m_right_pwm_l_chan = (m_right_pwm_l_pin >= 0) ? 3 : -1;

  // Configure PWM channels and pins using small helper functions
  if (m_left_pwm_r_chan >= 0) {
    configurePWMPin(m_left_pwm_r_pin, m_left_pwm_r_chan);
  }
  if (m_left_pwm_l_chan >= 0) {
    configurePWMPin(m_left_pwm_l_pin, m_left_pwm_l_chan);
  }
  if (m_right_pwm_r_chan >= 0) {
    configurePWMPin(m_right_pwm_r_pin, m_right_pwm_r_chan);
  }
  if (m_right_pwm_l_chan >= 0) {
    configurePWMPin(m_right_pwm_l_pin, m_right_pwm_l_chan);
  }

  // Configure enable pins as outputs and set low
  if (m_left_en_r_pin >= 0) {
    configureEnablePin(m_left_en_r_pin);
  }
  if (m_left_en_l_pin >= 0) {
    configureEnablePin(m_left_en_l_pin);
  }
  if (m_right_en_r_pin >= 0) {
    configureEnablePin(m_right_en_r_pin);
  }
  if (m_right_en_l_pin >= 0) {
    configureEnablePin(m_right_en_l_pin);
  }

  // Ensure all PWM channels are explicitly zeroed at init (defensive)
  if (m_left_pwm_r_chan >= 0) {
    setPWMduty(m_left_pwm_r_chan, 0);
  }
  if (m_left_pwm_l_chan >= 0) {
    setPWMduty(m_left_pwm_l_chan, 0);
  }
  if (m_right_pwm_r_chan >= 0) {
    setPWMduty(m_right_pwm_r_chan, 0);
  }
  if (m_right_pwm_l_chan >= 0) {
    setPWMduty(m_right_pwm_l_chan, 0);
  }

  // Configure encoders using PCNT hardware counters when available
#if DC_ENCODER_PRESENT_LEFT
  if (DC_LEFT_ENCODER_A_PIN >= 0 && DC_LEFT_ENCODER_B_PIN >= 0) {
    pinMode(DC_LEFT_ENCODER_A_PIN, INPUT_PULLUP);
    pinMode(DC_LEFT_ENCODER_B_PIN, INPUT_PULLUP);
    setup_pcnt_unit(m_left_pcnt_unit, m_left_pcnt_configured, PCNT_UNIT_0,
                    DC_LEFT_ENCODER_A_PIN, DC_LEFT_ENCODER_B_PIN);
  }
#endif
#if DC_ENCODER_PRESENT_RIGHT
  if (DC_RIGHT_ENCODER_A_PIN >= 0 && DC_RIGHT_ENCODER_B_PIN >= 0) {
    pinMode(DC_RIGHT_ENCODER_A_PIN, INPUT_PULLUP);
    pinMode(DC_RIGHT_ENCODER_B_PIN, INPUT_PULLUP);
    setup_pcnt_unit(m_right_pcnt_unit, m_right_pcnt_configured, PCNT_UNIT_1,
                    DC_RIGHT_ENCODER_A_PIN, DC_RIGHT_ENCODER_B_PIN);
  }
#endif

  LOG_PRINTLN(abbot::log::CHANNEL_MOTOR,
              "DCMirrorDriver: initMotorDriver() - hardware configured");
  LOG_PRINTF(
      abbot::log::CHANNEL_MOTOR,
      "DCMirrorDriver: PCNT configured left=%d unit=%d right=%d unit=%d\n",
      (int)m_left_pcnt_configured, m_left_pcnt_unit,
      (int)m_right_pcnt_configured, m_right_pcnt_unit);
}

void DCMirrorDriver::clearCommandState() {
  m_last_left_cmd = 0.0f;
  m_last_right_cmd = 0.0f;
  m_last_left_dir = 0;
  m_last_right_dir = 0;
}

float DCMirrorDriver::getLastMotorCommand(MotorSide side) {
  if (side == MotorSide::LEFT) {
    return m_last_left_cmd;
  }
  if (side == MotorSide::RIGHT) {
    return m_last_right_cmd;
  }
  return 0.0f;
}

void DCMirrorDriver::enableMotors() {
  m_enabled = true;
  // Defensive: ensure PWM channels are at 0 before enabling EN pins
  if (m_left_pwm_r_chan >= 0)
    setPWMduty(m_left_pwm_r_chan, 0);
  if (m_left_pwm_l_chan >= 0)
    setPWMduty(m_left_pwm_l_chan, 0);
  if (m_right_pwm_r_chan >= 0)
    setPWMduty(m_right_pwm_r_chan, 0);
  if (m_right_pwm_l_chan >= 0)
    setPWMduty(m_right_pwm_l_chan, 0);
  // Set all EN pins high to enable H-bridges
  if (m_left_en_r_pin >= 0)
    setEnablePinState(m_left_en_r_pin, true);
  if (m_left_en_l_pin >= 0)
    setEnablePinState(m_left_en_l_pin, true);
  if (m_right_en_r_pin >= 0)
    setEnablePinState(m_right_en_r_pin, true);
  if (m_right_en_l_pin >= 0)
    setEnablePinState(m_right_en_l_pin, true);
  LOG_PRINTLN(abbot::log::CHANNEL_MOTOR, "DCMirrorDriver: motors enabled");
}

void DCMirrorDriver::disableMotors() {
  m_enabled = false;
  // Stop PWM channels and clear EN pins using helper functions
  if (m_left_pwm_r_chan >= 0)
    setPWMduty(m_left_pwm_r_chan, 0);
  if (m_left_pwm_l_chan >= 0)
    setPWMduty(m_left_pwm_l_chan, 0);
  if (m_right_pwm_r_chan >= 0)
    setPWMduty(m_right_pwm_r_chan, 0);
  if (m_right_pwm_l_chan >= 0)
    setPWMduty(m_right_pwm_l_chan, 0);
  if (m_left_en_r_pin >= 0)
    setEnablePinState(m_left_en_r_pin, false);
  if (m_left_en_l_pin >= 0)
    setEnablePinState(m_left_en_l_pin, false);
  if (m_right_en_r_pin >= 0)
    setEnablePinState(m_right_en_r_pin, false);
  if (m_right_en_l_pin >= 0)
    setEnablePinState(m_right_en_l_pin, false);
  LOG_PRINTLN(abbot::log::CHANNEL_MOTOR,
              "DCMirrorDriver: motors disabled (hardware)");
}

bool DCMirrorDriver::areMotorsEnabled() { return m_enabled; }

void DCMirrorDriver::printStatus() {
  char buf[256];
  snprintf(buf, sizeof(buf),
           "DCMirrorDriver: enabled=%d left_cmd=%.3f right_cmd=%.3f "
           "left_enc=%lld right_enc=%lld",
           (int)m_enabled, (double)m_last_left_cmd, (double)m_last_right_cmd,
           (long long)m_left_encoder, (long long)m_right_encoder);
  LOG_PRINTLN(abbot::log::CHANNEL_MOTOR, buf);
  LOG_PRINTF(abbot::log::CHANNEL_MOTOR,
             "DCMirrorDriver: PCNT left_configured=%d left_unit=%d "
             "right_configured=%d right_unit=%d\n",
             (int)m_left_pcnt_configured, m_left_pcnt_unit,
             (int)m_right_pcnt_configured, m_right_pcnt_unit);
}

void DCMirrorDriver::dumpConfig() {
  LOG_PRINTLN(abbot::log::CHANNEL_MOTOR,
              "DCMirrorDriver: dumpConfig (pins and mirror settings)");
  char buf[192];
  snprintf(
      buf, sizeof(buf),
      "LEFT PWM R=%d L=%d EN_R=%d EN_L=%d RIGHT PWM R=%d L=%d EN_R=%d EN_L=%d",
      DC_LEFT_PWM_R_PIN, DC_LEFT_PWM_L_PIN, DC_LEFT_EN_R_PIN, DC_LEFT_EN_L_PIN,
      DC_RIGHT_PWM_R_PIN, DC_RIGHT_PWM_L_PIN, DC_RIGHT_EN_R_PIN,
      DC_RIGHT_EN_L_PIN);
  LOG_PRINTLN(abbot::log::CHANNEL_MOTOR, buf);
  snprintf(buf, sizeof(buf),
           "DC_MIRROR_MODE_ENABLED=%d LEFT_ENCODER_PRESENT=%d "
           "RIGHT_ENCODER_PRESENT=%d AUTH_SIDE_RIGHT=%d",
           DC_MIRROR_MODE_ENABLED, DC_ENCODER_PRESENT_LEFT,
           DC_ENCODER_PRESENT_RIGHT, DC_MIRROR_AUTH_SIDE_RIGHT);
  LOG_PRINTLN(abbot::log::CHANNEL_MOTOR, buf);
}

static inline int motorIdToIndex(int id) {
  if (id == DC_LEFT_MOTOR_ID)
    return 0;
  if (id == DC_RIGHT_MOTOR_ID)
    return 1;
  return -1;
}

void DCMirrorDriver::setMotorCommandBoth(float left_command,
                                         float right_command) {
  m_last_left_cmd = left_command;
  m_last_right_cmd = right_command;
  // record timestamp when the command was set (in microseconds)
  // Protect 64-bit writes with a critical section to avoid torn reads on
  // 32-bit architectures (ESP32). Use the per-instance mutex to keep
  // synchronization responsibility inside the driver object.
  portENTER_CRITICAL(&m_command_time_mux);
  m_last_left_command_time_us = (uint64_t)esp_timer_get_time();
  m_last_right_command_time_us = (uint64_t)esp_timer_get_time();
  portEXIT_CRITICAL(&m_command_time_mux);
  // Map normalized commands [-1.0..1.0] to raw speed and apply to hardware
  int16_t rawLeft =
      (int16_t)roundf(left_command * (float)DC_VELOCITY_MAX_SPEED);
  int16_t rawRight =
      (int16_t)roundf(right_command * (float)DC_VELOCITY_MAX_SPEED);
  applyHardwareCommand(MotorSide::LEFT, rawLeft);
  applyHardwareCommand(MotorSide::RIGHT, rawRight);

  // Update simulated encoders for compatibility with existing logic/tests
  const float scale =
      DC_VELOCITY_TARGET_INCREMENT_SCALE; // counts per cycle per unit command
  int32_t left_inc = (int32_t)roundf(left_command * scale);
  int32_t right_inc = (int32_t)roundf(right_command * scale);
  // Only apply simulated increments when a hardware encoder is NOT present
#if !DC_ENCODER_PRESENT_LEFT
  m_left_encoder += left_inc;
#endif
#if !DC_ENCODER_PRESENT_RIGHT
  m_right_encoder += right_inc;
#endif
  applyMirrorIfNeeded();
  checkDivergenceAndSafety();
}

void DCMirrorDriver::setMotorCommand(MotorSide side, float command) {
  if (side == MotorSide::LEFT) {
    setMotorCommandBoth(command, m_last_right_cmd);
  } else if (side == MotorSide::RIGHT) {
    setMotorCommandBoth(m_last_left_cmd, command);
  }
}

void DCMirrorDriver::setMotorCommandRaw(MotorSide side, int16_t rawSpeed) {
  // Apply raw speed directly to hardware and update last command
  if (side == MotorSide::LEFT) {
    m_last_left_cmd = (float)rawSpeed / (float)DC_VELOCITY_MAX_SPEED;
    portENTER_CRITICAL(&m_command_time_mux);
    m_last_left_command_time_us = (uint64_t)esp_timer_get_time();
    portEXIT_CRITICAL(&m_command_time_mux);
  } else {
    m_last_right_cmd = (float)rawSpeed / (float)DC_VELOCITY_MAX_SPEED;
    portENTER_CRITICAL(&m_command_time_mux);
    m_last_right_command_time_us = (uint64_t)esp_timer_get_time();
    portEXIT_CRITICAL(&m_command_time_mux);
  }
  applyHardwareCommand(side, rawSpeed);
}

int32_t DCMirrorDriver::readEncoder(MotorSide side) {
  // If encoder present for the requested side, return it; otherwise return
  // mirror of the authoritative encoder if mirror mode enabled.
  if (side == MotorSide::LEFT) {
    if (DC_ENCODER_PRESENT_LEFT) {
      if (m_left_pcnt_configured) {
        readAndAccumulatePCNT((pcnt_unit_t)m_left_pcnt_unit, m_left_encoder,
                              DC_ENCODER_SIGNALS_PER_PIN);
      } else {
        LOG_PRINTF(abbot::log::CHANNEL_MOTOR,
                   "DCMirrorDriver: readEncoder LEFT - PCNT not configured, "
                   "returning simulated=%lld\n",
                   (long long)m_left_encoder);
      }
      int32_t logicalPosition = (int32_t)m_left_encoder;
      // If motor is inverted, hardware encoder counts will be reversed relative
      // to logical forward. Invert here so that positive command = positive
      // encoder delta.
      if (isMotorInverted(MotorSide::LEFT)) {
        logicalPosition = -logicalPosition;
      }
      return logicalPosition;
    }
    if (DC_MIRROR_MODE_ENABLED) {
      // In mirror mode, we assume the logical position of the missing encoder
      // matches the logical position of the present one.
      if (DC_MIRROR_AUTH_SIDE_RIGHT && DC_ENCODER_PRESENT_RIGHT) {
        return readEncoder(MotorSide::RIGHT);
      }
    }
    return 0;
  } else if (side == MotorSide::RIGHT) {
    if (DC_ENCODER_PRESENT_RIGHT) {
      if (m_right_pcnt_configured) {
        readAndAccumulatePCNT((pcnt_unit_t)m_right_pcnt_unit, m_right_encoder,
                              DC_ENCODER_SIGNALS_PER_PIN);
      } else {
        LOG_PRINTF(abbot::log::CHANNEL_MOTOR,
                   "DCMirrorDriver: readEncoder RIGHT - PCNT not configured, "
                   "returning simulated=%lld\n",
                   (long long)m_right_encoder);
      }
      int32_t logicalPosition = (int32_t)m_right_encoder;
      if (isMotorInverted(MotorSide::RIGHT)) {
        logicalPosition = -logicalPosition;
      }
      return logicalPosition;
    }
    if (DC_MIRROR_MODE_ENABLED) {
      if (!DC_MIRROR_AUTH_SIDE_RIGHT && DC_ENCODER_PRESENT_LEFT) {
        return readEncoder(MotorSide::LEFT);
      }
    }
    return 0;
  }
  return 0;
}

void DCMirrorDriver::resetPositionTracking() {
  m_left_encoder = 0;
  m_right_encoder = 0;
#if DC_ENCODER_PRESENT_LEFT
  if (m_left_pcnt_configured && m_left_pcnt_unit >= 0)
    pcnt_counter_clear((pcnt_unit_t)m_left_pcnt_unit);
#endif
#if DC_ENCODER_PRESENT_RIGHT
  if (m_right_pcnt_configured && m_right_pcnt_unit >= 0)
    pcnt_counter_clear((pcnt_unit_t)m_right_pcnt_unit);
#endif
}

// Expose last command time for telemetry/diagnostics
uint64_t DCMirrorDriver::getLastCommandTimeUs(MotorSide side) const {
  uint64_t rv = 0;
  // Protect 64-bit reads with the same critical section used for writes.
  portENTER_CRITICAL(&m_command_time_mux);
  if (side == MotorSide::LEFT) {
    rv = m_last_left_command_time_us;
  } else {
    rv = m_last_right_command_time_us;
  }
  portEXIT_CRITICAL(&m_command_time_mux);
  return rv;
}

// Reset internal speed estimator state for both sides
void DCMirrorDriver::resetSpeedEstimator() {
  m_left_estimator.reset();
  m_right_estimator.reset();
}

bool DCMirrorDriver::processSerialCommand(const String &line) {
  // Minimal command support: DUMP, POS, RESETPOS. For other textual
  // motor commands (e.g. "MOTOR ENABLE", "MOTOR SET ...") defer to the
  // shared motor command parser used by the servo adapter so behaviour is
  // consistent across drivers.
  String u = line;
  u.trim();
  u.toUpperCase();
  if (u == "DUMP") {
    dumpConfig();
    return true;
  }
  if (u == "POS") {
    printStatus();
    return true;
  }
  if (u == "RESETPOS" || u == "RESETPOS\r") {
    resetPositionTracking();
    LOG_PRINTLN(abbot::log::CHANNEL_MOTOR,
                "DCMirrorDriver: resetPositionTracking");
    return true;
  }

  // DC-specific: support runtime inversion toggle
  // Syntax: MOTOR INVERT <LEFT|RIGHT|ID> [0|1]
  {
    String t = u;
    char buf[128];
    t.toCharArray(buf, sizeof(buf));
    char *tk = strtok(buf, " \t\r\n");
    if (tk && strcmp(tk, "MOTOR") == 0) {
      char *cmd = strtok(NULL, " \t\r\n");
      if (cmd && strcmp(cmd, "INVERT") == 0) {
        char *arg = strtok(NULL, " \t\r\n");
        if (!arg)
          return false;
        int sideIdx = -1;
        if (strcmp(arg, "LEFT") == 0)
          sideIdx = 0;
        else if (strcmp(arg, "RIGHT") == 0)
          sideIdx = 1;
        else {
          int id = atoi(arg);
          sideIdx = (id == getMotorId(MotorSide::LEFT))
                        ? 0
                        : ((id == getMotorId(MotorSide::RIGHT)) ? 1 : -1);
        }
        if (sideIdx == -1)
          return false;
        char *val = strtok(NULL, " \t\r\n");
        bool newval = true;
        bool hasVal = false;
        if (val) {
          int v = atoi(val);
          newval = (v != 0);
          hasVal = true;
        }
        if (sideIdx == 0) {
          m_left_invert_override_enabled = true;
          m_left_invert_override_value = newval;
          LOG_PRINTF(abbot::log::CHANNEL_MOTOR,
                     "DCMirrorDriver: LEFT invert override=%d\n", (int)newval);
        } else {
          m_right_invert_override_enabled = true;
          m_right_invert_override_value = newval;
          LOG_PRINTF(abbot::log::CHANNEL_MOTOR,
                     "DCMirrorDriver: RIGHT invert override=%d\n", (int)newval);
        }
        return true;
      }
    }
  }

  // Fallback to the shared base-class parser (handles MOTOR ENABLE/SET/etc.)
  return AbstractMotorDriver::processSerialCommand(line);
}

// Configuration/query implementations for DC mirror driver
int DCMirrorDriver::getMotorId(MotorSide side) const {
  return (side == MotorSide::LEFT) ? DC_LEFT_MOTOR_ID : DC_RIGHT_MOTOR_ID;
}

bool DCMirrorDriver::isMotorInverted(MotorSide side) const {
  if (side == MotorSide::LEFT) {
    if (m_left_invert_override_enabled) {
      return m_left_invert_override_value;
    }
    return (DC_LEFT_MOTOR_INVERT != 0);
  } else {
    if (m_right_invert_override_enabled) {
      return m_right_invert_override_value;
    }
    return (DC_RIGHT_MOTOR_INVERT != 0);
  }
}

float DCMirrorDriver::getVelocityMaxSpeed() const {
  return (float)DC_VELOCITY_MAX_SPEED;
}

float DCMirrorDriver::getVelocityTargetIncrementScale() const {
  return (float)DC_VELOCITY_TARGET_INCREMENT_SCALE;
}

float DCMirrorDriver::getVelocityPositionKp() const {
  return 0.0f;
}

const char *DCMirrorDriver::getDriverName() const {
  return "dc_mirror";
}

void DCMirrorDriver::applyMirrorIfNeeded() {
  if (!DC_MIRROR_MODE_ENABLED) {
    return;
  }
  // If left missing and right present, mirror right -> left
  if (!DC_ENCODER_PRESENT_LEFT && DC_ENCODER_PRESENT_RIGHT) {
    int32_t logicalPosition = (int32_t)m_right_encoder;
    if (DC_LEFT_MOTOR_INVERT != DC_RIGHT_MOTOR_INVERT) {
      logicalPosition = -logicalPosition;
    }
    m_left_encoder = logicalPosition;
  }
  // If right missing and left present and auth side left, mirror left -> right
  if (!DC_ENCODER_PRESENT_RIGHT && DC_ENCODER_PRESENT_LEFT) {
    int32_t logicalPosition = (int32_t)m_left_encoder;
    if (DC_LEFT_MOTOR_INVERT != DC_RIGHT_MOTOR_INVERT) {
      logicalPosition = -logicalPosition;
    }
    m_right_encoder = logicalPosition;
  }
}

void DCMirrorDriver::checkDivergenceAndSafety() {
  if (!DC_MIRROR_MODE_ENABLED) {
    return;
  }
  // If both encoders present, check divergence; else skip
  if (DC_ENCODER_PRESENT_LEFT && DC_ENCODER_PRESENT_RIGHT) {
    int64_t diff = llabs(m_left_encoder - m_right_encoder);
    if (diff > DC_MIRROR_DIVERGENCE_THRESH) {
      char buf[128];
      snprintf(buf, sizeof(buf),
               "DCMirrorDriver: encoder divergence detected=%lld > %d - "
               "disabling motors",
               (long long)diff, (int)DC_MIRROR_DIVERGENCE_THRESH);
      LOG_PRINTLN(abbot::log::CHANNEL_MOTOR, buf);
      disableMotors();
    }
  }
}

// Hardware helpers
void DCMirrorDriver::stopMotorHardware(MotorSide side) {
#if DC_MIRROR_DRIVER_DEBUG
  const char *sname = (side == MotorSide::LEFT) ? "LEFT" : "RIGHT";
  LOG_PRINTF(abbot::log::CHANNEL_MOTOR, "DCMirrorDriver: stopMotorHardware(%s)",
             sname);
#endif
  if (side == MotorSide::LEFT) {
    if (m_left_pwm_r_chan >= 0)
      setPWMduty(m_left_pwm_r_chan, 0);
    if (m_left_pwm_l_chan >= 0)
      setPWMduty(m_left_pwm_l_chan, 0);
    if (m_left_en_r_pin >= 0)
      setEnablePinState(m_left_en_r_pin, false);
    if (m_left_en_l_pin >= 0)
      setEnablePinState(m_left_en_l_pin, false);
  } else {
    if (m_right_pwm_r_chan >= 0)
      setPWMduty(m_right_pwm_r_chan, 0);
    if (m_right_pwm_l_chan >= 0)
      setPWMduty(m_right_pwm_l_chan, 0);
    if (m_right_en_r_pin >= 0)
      setEnablePinState(m_right_en_r_pin, false);
    if (m_right_en_l_pin >= 0)
      setEnablePinState(m_right_en_l_pin, false);
  }
}

void DCMirrorDriver::applyHardwareCommand(MotorSide side, int16_t rawSpeed) {
  // Log command entry timestamp for latency measurement
  uint64_t cmd_entry_time = (uint64_t)esp_timer_get_time();
  const char *sname = (side == MotorSide::LEFT) ? "LEFT" : "RIGHT";
  LOG_PRINTF(abbot::log::CHANNEL_MOTOR,
             "DCMirrorDriver: command entry %s at %llu us (raw=%d)",
             sname, (unsigned long long)cmd_entry_time, (int)rawSpeed);

  // Normalize by inversion setting
  bool inverted = isMotorInverted(side);
  int32_t desired = rawSpeed;
  if (inverted) {
    desired = -desired;
  }

  // Determine direction and duty
  int32_t absv = (desired >= 0) ? desired : -desired;
  if (absv > DC_VELOCITY_MAX_SPEED) {
    absv = DC_VELOCITY_MAX_SPEED;
  }
  uint32_t duty = 0;
  if (DC_VELOCITY_MAX_SPEED > 0) {
    duty = (uint32_t)(((uint64_t)absv * (uint64_t)m_pwm_max_duty) /
                      (uint64_t)DC_VELOCITY_MAX_SPEED);
  }

  // Determine new direction: -1, 0, +1
  int newDir = (desired > 0) ? 1 : ((desired < 0) ? -1 : 0);
  int *pLastDir =
      (side == MotorSide::LEFT) ? &m_last_left_dir : &m_last_right_dir;

#if DC_MIRROR_DRIVER_DEBUG
  LOG_PRINTF(abbot::log::CHANNEL_MOTOR,
             "DCMirrorDriver: applyHardwareCommand(%s) raw=%d desired=%d "
             "duty=%u lastDir=%d",
             sname, (int)rawSpeed, (int)desired, (unsigned)duty, *pLastDir);
#endif

  // If new command requests stop, perform stop and clear direction
  if (newDir == 0 || duty == 0) {
    stopMotorHardware(side);
    *pLastDir = 0;
    return;
  }

  // If direction unchanged, just update duty on the active channel and ensure
  // EN pins
  if (*pLastDir == newDir) {
#if DC_MIRROR_DRIVER_DEBUG
    LOG_PRINTF(
        abbot::log::CHANNEL_MOTOR,
        "DCMirrorDriver: direction unchanged for %s (dir=%d) - updating duty",
        sname, newDir);
#endif
    applyDutyForSide(side, desired, duty);
    *pLastDir = newDir;
    return;
  }

// Direction changed: perform safe stop, small deadtime, then apply new channel
#if DC_MIRROR_DRIVER_DEBUG
  LOG_PRINTF(abbot::log::CHANNEL_MOTOR,
             "DCMirrorDriver: direction change for %s: %d -> %d - performing "
             "safe stop",
             sname, *pLastDir, newDir);
#endif
  stopMotorHardware(side);
  safeDirectionChangeDelay();

  applyDutyForSide(side, desired, duty);

#if DC_MIRROR_DRIVER_DEBUG
  LOG_PRINTF(abbot::log::CHANNEL_MOTOR,
             "DCMirrorDriver: applied new direction %d for %s", newDir, sname);
#endif

  *pLastDir = newDir;
}

// Small, well-named hardware helper implementations
void DCMirrorDriver::configurePWMPin(int pin, int chan) {
  // Configure LEDC channel and attach pin, leave duty at 0
  ledcSetup(chan, DC_PWM_FREQUENCY_HZ, m_pwm_resolution_bits);
  ledcAttachPin(pin, chan);
  ledcWrite(chan, 0);
}

void DCMirrorDriver::configureEnablePin(int pin) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
}

void DCMirrorDriver::setPWMduty(int chan, uint32_t duty) {
  if (chan < 0) {
    return;
  }
  if (duty > (uint32_t)m_pwm_max_duty) {
    duty = m_pwm_max_duty;
  }
  ledcWrite(chan, duty);
}

void DCMirrorDriver::setEnablePinState(int pin, bool high) {
  if (pin < 0) {
    return;
  }
  digitalWrite(pin, high ? HIGH : LOW);
}

void DCMirrorDriver::safeDirectionChangeDelay() {
  // Small pause to avoid shoot-through when changing direction (configurable)
  delay(DC_DIRECTION_CHANGE_DELAY_MS);
}

// Apply PWM duty to the correct channel for the given side and ensure EN pins
void DCMirrorDriver::applyDutyForSide(MotorSide side, int32_t desired,
                                      uint32_t duty) {
  if (side == MotorSide::LEFT) {
    if (desired > 0) {
      if (m_left_pwm_r_chan >= 0) {
        setPWMduty(m_left_pwm_r_chan, duty);
#if DC_MIRROR_DRIVER_DEBUG
        LOG_PRINTF(abbot::log::CHANNEL_MOTOR,
                   "DCMirrorDriver: LEFT -> using R channel chan=%d duty=%u",
                   m_left_pwm_r_chan, duty);
#endif
      }
    } else {
      if (m_left_pwm_l_chan >= 0) {
        setPWMduty(m_left_pwm_l_chan, duty);
#if DC_MIRROR_DRIVER_DEBUG
        LOG_PRINTF(abbot::log::CHANNEL_MOTOR,
                   "DCMirrorDriver: LEFT -> using L channel chan=%d duty=%u",
                   m_left_pwm_l_chan, duty);
#endif
      }
    }
    if (m_left_en_r_pin >= 0) {
      setEnablePinState(m_left_en_r_pin, true);
    }
    if (m_left_en_l_pin >= 0) {
      setEnablePinState(m_left_en_l_pin, true);
    }
    // record precise timestamp when hardware command became active
    m_last_left_command_time_us = (uint64_t)esp_timer_get_time();
#if DC_MIRROR_DRIVER_DEBUG
    LOG_PRINTF(abbot::log::CHANNEL_MOTOR,
               "DCMirrorDriver: LEFT EN pins set R=%d L=%d", m_left_en_r_pin,
               m_left_en_l_pin);
#endif
  } else {
    if (desired > 0) {
      if (m_right_pwm_r_chan >= 0) {
        setPWMduty(m_right_pwm_r_chan, duty);
#if DC_MIRROR_DRIVER_DEBUG
        LOG_PRINTF(abbot::log::CHANNEL_MOTOR,
                   "DCMirrorDriver: RIGHT -> using R channel chan=%d duty=%u",
                   m_right_pwm_r_chan, duty);
#endif
      }
    } else {
      if (m_right_pwm_l_chan >= 0) {
        setPWMduty(m_right_pwm_l_chan, duty);
#if DC_MIRROR_DRIVER_DEBUG
        LOG_PRINTF(abbot::log::CHANNEL_MOTOR,
                   "DCMirrorDriver: RIGHT -> using L channel chan=%d duty=%u",
                   m_right_pwm_l_chan, duty);
#endif
      }
    }
    if (m_right_en_r_pin >= 0) {
      setEnablePinState(m_right_en_r_pin, true);
    }
    if (m_right_en_l_pin >= 0) {
      setEnablePinState(m_right_en_l_pin, true);
    }
    // record precise timestamp when hardware command became active
    m_last_right_command_time_us = (uint64_t)esp_timer_get_time();
#if DC_MIRROR_DRIVER_DEBUG
    LOG_PRINTF(abbot::log::CHANNEL_MOTOR,
               "DCMirrorDriver: RIGHT EN pins set R=%d L=%d", m_right_en_r_pin,
               m_right_en_l_pin);
#endif
  }
}

// Factory helper
// Encoder increment helpers called from ISRs
void DCMirrorDriver::incrementLeftEncoder(int delta) {
  portENTER_CRITICAL_ISR(&s_encoder_mux);
  m_left_encoder += delta;
  portEXIT_CRITICAL_ISR(&s_encoder_mux);
}

void DCMirrorDriver::incrementRightEncoder(int delta) {
  portENTER_CRITICAL_ISR(&s_encoder_mux);
  m_right_encoder += delta;
  portEXIT_CRITICAL_ISR(&s_encoder_mux);
}

// Read a filtered speed estimate (counts/sec) for the requested side.
// Delegates filtering to the SpeedEstimator helper (single responsibility).
float DCMirrorDriver::readSpeed(MotorSide side) {
  uint64_t now = (uint64_t)esp_timer_get_time();
  if (side == MotorSide::LEFT) {
    int32_t enc = readEncoder(MotorSide::LEFT);
    int64_t curCount = (int64_t)enc;
    return m_left_estimator.update(curCount, now);
  } else {
    int32_t enc = readEncoder(MotorSide::RIGHT);
    int64_t curCount = (int64_t)enc;
    return m_right_estimator.update(curCount, now);
  }
}

// Global instance used by ISRs
static DCMirrorDriver g_dcMirrorDriver;

// Using PCNT hardware counters; no software ISRs are used here.

void installDefaultDCMirrorDriver() { setActiveMotorDriver(&g_dcMirrorDriver); }

} // namespace motor
} // namespace abbot
