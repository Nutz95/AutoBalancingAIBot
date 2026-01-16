// abstract_motor_driver.cpp
#include "../include/motor_drivers/AbstractMotorDriver.h"
#include <cstring>
#include <cctype>
#if defined(UNIT_TEST_HOST)
#include <string>
#include <algorithm>
#endif

namespace abbot {
namespace motor {

bool AbstractMotorDriver::processSerialCommand(const String &line) {
  if (line.length() == 0)
    return false;

#if defined(UNIT_TEST_HOST)
  std::string up = line;
  // trim
  up.erase(up.begin(), std::find_if(up.begin(), up.end(), [](unsigned char ch) { return !std::isspace(ch); }));
  up.erase(std::find_if(up.rbegin(), up.rend(), [](unsigned char ch) { return !std::isspace(ch); }).base(), up.end());
  // uppercase
  std::transform(up.begin(), up.end(), up.begin(), [](unsigned char c) { return (char)std::toupper(c); });
  char buf[128];
  std::strncpy(buf, up.c_str(), sizeof(buf));
  buf[sizeof(buf)-1] = '\0';
#else
  String up = line;
  up.trim();
  up.toUpperCase();
  char buf[128];
  up.toCharArray(buf, sizeof(buf));
#endif
  char *tk = strtok(buf, " \t\r\n");
  if (!tk)
    return false;
  char *cmd = nullptr;
  if (strcmp(tk, "MOTOR") == 0) {
    cmd = strtok(NULL, " \t\r\n");
  } else {
    cmd = tk;
  }
  if (!cmd)
    return false;

  if (strcmp(cmd, "ENABLE") == 0) {
    enableMotors();
    printStatus();
    return true;
  }
  if (strcmp(cmd, "DISABLE") == 0) {
    disableMotors();
    return true;
  }
  if (strcmp(cmd, "STATUS") == 0) {
    printStatus();
    return true;
  }
  if (strcmp(cmd, "SET") == 0) {
    char *arg = strtok(NULL, " \t\r\n");
    if (!arg)
      return false;
    MotorSide side;
    bool sideOk = true;
    if (strcmp(arg, "LEFT") == 0)
      side = MotorSide::LEFT;
    else if (strcmp(arg, "RIGHT") == 0)
      side = MotorSide::RIGHT;
    else {
      int id = atoi(arg);
      if (id == getMotorId(MotorSide::LEFT))
        side = MotorSide::LEFT;
      else if (id == getMotorId(MotorSide::RIGHT))
        side = MotorSide::RIGHT;
      else
        sideOk = false;
    }
    if (!sideOk)
      return false;
    char *v = strtok(NULL, " \t\r\n");
    if (!v)
      return false;
    if (strcmp(v, "RAW") == 0) {
      char *rv = strtok(NULL, " \t\r\n");
      if (!rv)
        return false;
      int16_t rawv = (int16_t)atoi(rv);
      setMotorCommandRaw(side, rawv);
      return true;
    }
    float cmdv = atof(v);
    if (cmdv > 1.0f || cmdv < -1.0f)
      cmdv = cmdv / 100.0f;
    setMotorCommand(side, cmdv);
    return true;
  }
  if (strcmp(cmd, "READ") == 0) {
    char *arg = strtok(NULL, " \t\r\n");
    if (!arg)
      return false;
    MotorSide side;
    bool sideOk = true;
    if (strcmp(arg, "LEFT") == 0)
      side = MotorSide::LEFT;
    else if (strcmp(arg, "RIGHT") == 0)
      side = MotorSide::RIGHT;
    else {
      int id = atoi(arg);
      if (id == getMotorId(MotorSide::LEFT))
        side = MotorSide::LEFT;
      else if (id == getMotorId(MotorSide::RIGHT))
        side = MotorSide::RIGHT;
      else
        sideOk = false;
    }
    if (!sideOk)
      return false;
    int32_t val = readEncoder(side);
    LOG_PRINTF(::abbot::log::CHANNEL_MOTOR,
               "motor_driver: encoder side=%d val=%ld\n", (int)side,
               (long)val);
    return true;
  }

  if (strcmp(cmd, "PARAMS") == 0) {
    dumpConfig();
    return true;
  }

  return false;
}


// --- Non-inline defaults moved from header to this translation unit ---

float AbstractMotorDriver::readSpeed(MotorSide /*side*/) {
  return 0.0f;
}

void AbstractMotorDriver::readEncodersBoth(int32_t &left, int32_t &right) {
  left = readEncoder(MotorSide::LEFT);
  right = readEncoder(MotorSide::RIGHT);
}

uint64_t AbstractMotorDriver::getLastCommandTimeUs(MotorSide /*side*/) const {
  return 0;
}

} // namespace motor
} // namespace abbot
