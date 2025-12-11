// dc_mirror_driver.cpp
#include "../include/motor_drivers/dc_mirror_driver.h"
#include "../include/motor_drivers/driver_manager.h"
#include "../config/motor_configs/motor_common_config.h"
#include "../config/motor_configs/dc_motor_config.h"
#include "logging.h"

namespace abbot {
namespace motor {

DCMirrorDriver::DCMirrorDriver()
  : m_enabled(false), m_last_left_cmd(0.0f), m_last_right_cmd(0.0f), m_left_encoder(0), m_right_encoder(0) {}

void DCMirrorDriver::initMotorDriver() {
  // For a hardware driver we would configure PWM channels and enable pins here.
  // For this skeleton we just initialize state.
  m_enabled = false;
  m_left_encoder = 0;
  m_right_encoder = 0;
  m_last_left_cmd = 0.0f;
  m_last_right_cmd = 0.0f;
  LOG_PRINTLN(abbot::log::CHANNEL_MOTOR, "DCMirrorDriver: initMotorDriver()");
}

void DCMirrorDriver::clearCommandState() {
  m_last_left_cmd = 0.0f;
  m_last_right_cmd = 0.0f;
}

float DCMirrorDriver::getLastMotorCommand(MotorSide side) {
  if (side == MotorSide::LEFT) return m_last_left_cmd;
  if (side == MotorSide::RIGHT) return m_last_right_cmd;
  return 0.0f;
}

void DCMirrorDriver::enableMotors() {
  m_enabled = true;
  LOG_PRINTLN(abbot::log::CHANNEL_MOTOR, "DCMirrorDriver: motors enabled");
}

void DCMirrorDriver::disableMotors() {
  m_enabled = false;
  // Hardware should set PWM=0 here to avoid sign-change events causing shoot-through
  LOG_PRINTLN(abbot::log::CHANNEL_MOTOR, "DCMirrorDriver: motors disabled");
}

bool DCMirrorDriver::areMotorsEnabled() {
  return m_enabled;
}

void DCMirrorDriver::printStatus() {
  char buf[256];
  snprintf(buf, sizeof(buf), "DCMirrorDriver: enabled=%d left_cmd=%.3f right_cmd=%.3f left_enc=%lld right_enc=%lld",
           (int)m_enabled, (double)m_last_left_cmd, (double)m_last_right_cmd, (long long)m_left_encoder, (long long)m_right_encoder);
  LOG_PRINTLN(abbot::log::CHANNEL_MOTOR, buf);
}

void DCMirrorDriver::dumpConfig() {
  LOG_PRINTLN(abbot::log::CHANNEL_MOTOR, "DCMirrorDriver: dumpConfig (pins and mirror settings)");
  char buf[192];
  snprintf(buf, sizeof(buf), "LEFT PWM A=%d B=%d ENA=%d ENB=%d RIGHT PWM A=%d B=%d ENA=%d ENB=%d",
           DC_LEFT_PWM_A_PIN, DC_LEFT_PWM_B_PIN, DC_LEFT_EN_A_PIN, DC_LEFT_EN_B_PIN,
           DC_RIGHT_PWM_A_PIN, DC_RIGHT_PWM_B_PIN, DC_RIGHT_EN_A_PIN, DC_RIGHT_EN_B_PIN);
  LOG_PRINTLN(abbot::log::CHANNEL_MOTOR, buf);
  snprintf(buf, sizeof(buf), "DC_MIRROR_MODE_ENABLED=%d LEFT_ENCODER_PRESENT=%d RIGHT_ENCODER_PRESENT=%d AUTH_SIDE_RIGHT=%d",
           DC_MIRROR_MODE_ENABLED, DC_ENCODER_PRESENT_LEFT, DC_ENCODER_PRESENT_RIGHT, DC_MIRROR_AUTH_SIDE_RIGHT);
  LOG_PRINTLN(abbot::log::CHANNEL_MOTOR, buf);
}

static inline int motorIdToIndex(int id) {
  if (id == DC_LEFT_MOTOR_ID) return 0;
  if (id == DC_RIGHT_MOTOR_ID) return 1;
  return -1;
}

void DCMirrorDriver::setMotorCommandBoth(float left_command, float right_command) {
  m_last_left_cmd = left_command;
  m_last_right_cmd = right_command;
  // Simple state update: simulate encoder movement proportional to command
  // This helps host tests and allows mirror reads to return plausible values.
  const float scale = DC_VELOCITY_TARGET_INCREMENT_SCALE; // counts per cycle per unit command
  int32_t left_inc = (int32_t)roundf(left_command * scale);
  int32_t right_inc = (int32_t)roundf(right_command * scale);
  m_left_encoder += left_inc;
  m_right_encoder += right_inc;
  // If mirror mode enabled, enforce mirror policy
  applyMirrorIfNeeded();
  checkDivergenceAndSafety();
}

void DCMirrorDriver::setMotorCommand(MotorSide side, float command) {
  if (side == MotorSide::LEFT) setMotorCommandBoth(command, m_last_right_cmd);
  else if (side == MotorSide::RIGHT) setMotorCommandBoth(m_last_left_cmd, command);
}

void DCMirrorDriver::setMotorCommandRaw(MotorSide side, int16_t rawSpeed) {
  // For simplicity map rawSpeed -> normalized command
  float cmd = 0.0f;
  if (rawSpeed > 0) cmd = (float)rawSpeed / (float)DC_VELOCITY_MAX_SPEED;
  if (rawSpeed < 0) cmd = (float)rawSpeed / (float)DC_VELOCITY_MAX_SPEED;
  setMotorCommand(side, cmd);
}

int32_t DCMirrorDriver::readEncoder(MotorSide side) {
  // If encoder present for the requested side, return it; otherwise return
  // mirror of the authoritative encoder if mirror mode enabled.
  if (side == MotorSide::LEFT) {
    if (DC_ENCODER_PRESENT_LEFT) return (int32_t)m_left_encoder;
    if (DC_MIRROR_MODE_ENABLED) {
      // mirror from auth side
      if (DC_MIRROR_AUTH_SIDE_RIGHT && DC_ENCODER_PRESENT_RIGHT) {
        int32_t val = (int32_t)m_right_encoder;
        // apply inversion if motor directions differ
        if (DC_LEFT_MOTOR_INVERT != DC_RIGHT_MOTOR_INVERT) val = -val;
        return val;
      }
    }
    return 0;
  } else if (side == MotorSide::RIGHT) {
    if (DC_ENCODER_PRESENT_RIGHT) return (int32_t)m_right_encoder;
    if (DC_MIRROR_MODE_ENABLED) {
      if (!DC_MIRROR_AUTH_SIDE_RIGHT && DC_ENCODER_PRESENT_LEFT) {
        int32_t val = (int32_t)m_left_encoder;
        if (DC_LEFT_MOTOR_INVERT != DC_RIGHT_MOTOR_INVERT) val = -val;
        return val;
      }
      // If auth side is right but right encoder missing, we can't mirror
    }
    return 0;
  }
  return 0;
}

void DCMirrorDriver::resetPositionTracking() {
  m_left_encoder = 0;
  m_right_encoder = 0;
}

bool DCMirrorDriver::processSerialCommand(const String &line) {
  // Minimal command support: DUMP, POS, RESETPOS
  String u = line;
  u.trim();
  u.toUpperCase();
  if (u == "DUMP") { dumpConfig(); return true; }
  if (u == "POS") { printStatus(); return true; }
  if (u == "RESETPOS" || u == "RESETPOS\r" ) { resetPositionTracking(); LOG_PRINTLN(abbot::log::CHANNEL_MOTOR, "DCMirrorDriver: resetPositionTracking"); return true; }
  return false;
}

// Configuration/query implementations for DC mirror driver
int DCMirrorDriver::getMotorId(MotorSide side) const {
  return (side == MotorSide::LEFT) ? DC_LEFT_MOTOR_ID : DC_RIGHT_MOTOR_ID;
}

bool DCMirrorDriver::isMotorInverted(MotorSide side) const {
  return (side == MotorSide::LEFT) ? (DC_LEFT_MOTOR_INVERT != 0) : (DC_RIGHT_MOTOR_INVERT != 0);
}

float DCMirrorDriver::getVelocityMaxSpeed() const { return (float)DC_VELOCITY_MAX_SPEED; }
float DCMirrorDriver::getVelocityTargetIncrementScale() const { return (float)DC_VELOCITY_TARGET_INCREMENT_SCALE; }
float DCMirrorDriver::getVelocityPositionKp() const { return 0.0f; }

const char* DCMirrorDriver::getDriverName() const { return "dc_mirror"; }

void DCMirrorDriver::applyMirrorIfNeeded() {
  if (!DC_MIRROR_MODE_ENABLED) return;
  // If left missing and right present, mirror right -> left
  if (!DC_ENCODER_PRESENT_LEFT && DC_ENCODER_PRESENT_RIGHT) {
    int32_t val = (int32_t)m_right_encoder;
    if (DC_LEFT_MOTOR_INVERT != DC_RIGHT_MOTOR_INVERT) val = -val;
    m_left_encoder = val;
  }
  // If right missing and left present and auth side left, mirror left -> right
  if (!DC_ENCODER_PRESENT_RIGHT && DC_ENCODER_PRESENT_LEFT) {
    int32_t val = (int32_t)m_left_encoder;
    if (DC_LEFT_MOTOR_INVERT != DC_RIGHT_MOTOR_INVERT) val = -val;
    m_right_encoder = val;
  }
}

void DCMirrorDriver::checkDivergenceAndSafety() {
  if (!DC_MIRROR_MODE_ENABLED) return;
  // If both encoders present, check divergence; else skip
  if (DC_ENCODER_PRESENT_LEFT && DC_ENCODER_PRESENT_RIGHT) {
    int64_t diff = llabs(m_left_encoder - m_right_encoder);
    if (diff > DC_MIRROR_DIVERGENCE_THRESH) {
      char buf[128];
      snprintf(buf, sizeof(buf), "DCMirrorDriver: encoder divergence detected=%lld > %d - disabling motors",
               (long long)diff, (int)DC_MIRROR_DIVERGENCE_THRESH);
      LOG_PRINTLN(abbot::log::CHANNEL_MOTOR, buf);
      disableMotors();
    }
  }
}

// Factory helper
static DCMirrorDriver g_dcMirrorDriver;

void installDefaultDCMirrorDriver() {
  setActiveMotorDriver(&g_dcMirrorDriver);
}

} // namespace motor
} // namespace abbot
