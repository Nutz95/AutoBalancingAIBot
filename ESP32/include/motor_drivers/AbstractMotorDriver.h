// AbstractMotorDriver.h
#pragma once

#include "IMotorDriver.h"
#include "logging.h"

namespace abbot {
namespace motor {

// Base class that implements common textual serial command parsing for
// motor drivers (MOTOR ENABLE, MOTOR SET, MOTOR READ, etc.). Drivers can
// inherit from this to get the shared parsing while implementing the
// virtual hardware methods declared in IMotorDriver.
class AbstractMotorDriver : public IMotorDriver {
public:
  bool processSerialCommand(const String &line) override {
    if (line.length() == 0) return false;
    // Tokenize a local, upper-cased copy for case-insensitive parsing
    String up = line;
    up.trim();
    up.toUpperCase();
    char buf[128];
    up.toCharArray(buf, sizeof(buf));
    char *tk = strtok(buf, " \t\r\n");
    if (!tk) return false;
    // Accept either full form starting with "MOTOR" or direct commands
    char *cmd = nullptr;
    if (strcmp(tk, "MOTOR") == 0) {
      cmd = strtok(NULL, " \t\r\n");
    } else {
      // First token is the command itself (e.g. "POS", "PARAMS", "SET")
      cmd = tk;
    }
    if (!cmd) return false;

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
      if (!arg) return false;
      MotorSide side;
      bool sideOk = true;
      if (strcmp(arg, "LEFT") == 0) side = MotorSide::LEFT;
      else if (strcmp(arg, "RIGHT") == 0) side = MotorSide::RIGHT;
      else {
        int id = atoi(arg);
        if (id == getMotorId(MotorSide::LEFT)) side = MotorSide::LEFT;
        else if (id == getMotorId(MotorSide::RIGHT)) side = MotorSide::RIGHT;
        else sideOk = false;
      }
      if (!sideOk) return false;
      char *v = strtok(NULL, " \t\r\n");
      if (!v) return false;
      if (strcmp(v, "RAW") == 0) {
        char *rv = strtok(NULL, " \t\r\n");
        if (!rv) return false;
        int16_t rawv = (int16_t)atoi(rv);
        setMotorCommandRaw(side, rawv);
        return true;
      }
      float cmdv = atof(v);
      if (cmdv > 1.0f || cmdv < -1.0f) cmdv = cmdv / 100.0f;
      setMotorCommand(side, cmdv);
      return true;
    }
    if (strcmp(cmd, "READ") == 0) {
      char *arg = strtok(NULL, " \t\r\n");
      if (!arg) return false;
      MotorSide side;
      bool sideOk = true;
      if (strcmp(arg, "LEFT") == 0) side = MotorSide::LEFT;
      else if (strcmp(arg, "RIGHT") == 0) side = MotorSide::RIGHT;
      else {
        int id = atoi(arg);
        if (id == getMotorId(MotorSide::LEFT)) side = MotorSide::LEFT;
        else if (id == getMotorId(MotorSide::RIGHT)) side = MotorSide::RIGHT;
        else sideOk = false;
      }
      if (!sideOk) return false;
      int32_t val = readEncoder(side);
      LOG_PRINTF(abbot::log::CHANNEL_MOTOR, "motor_driver: encoder side=%d val=%ld\n", (int)side, (long)val);
      return true;
    }

    if (strcmp(cmd, "PARAMS") == 0) {
      // Drivers may override for richer behaviour; provide a generic dump
      dumpConfig();
      return true;
    }

    return false;
  }
};

} // namespace motor
} // namespace abbot
