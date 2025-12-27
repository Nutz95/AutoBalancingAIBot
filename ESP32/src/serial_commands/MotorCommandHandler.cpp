#include "serial_commands/MotorCommandHandler.h"
#include "motor_drivers/driver_manager.h"
#include "serial_commands/motor_telemetry_manager.h"
#include "logging.h"
#include "../../config/motor_configs/servo_motor_config.h" // For LEFT_MOTOR_ID, RIGHT_MOTOR_ID

namespace abbot {
namespace serialcmds {

bool MotorCommandHandler::handleCommand(const String& line, const String& lineUpper) {
    if (!lineUpper.startsWith("MOTOR")) {
        return false;
    }

    if (handleMotorGetEncoder(line, lineUpper)) return true;
    if (handleMotorTelemetry(line, lineUpper)) return true;

    // Fallback for other MOTOR commands that might be handled by the driver directly
    auto d = abbot::motor::getActiveMotorDriver();
    if (d) {
        // Extract the part after "MOTOR "
        int spaceIdx = lineUpper.indexOf(' ');
        if (spaceIdx != -1) {
            String subCmd = line.substring(spaceIdx + 1);
            subCmd.trim();
            // Some commands are handled by the driver's processSerialCommand
            // We return true if we think we've handled it or if it's a known command.
            // For now, we'll let the dispatcher know we've "seen" MOTOR.
        }
    }

    return false; // Let the dispatcher know we didn't fully consume it if it's not a specific sub-command
}

void MotorCommandHandler::motorSetHandler(const String &p) {
  String s = p;
  s.trim();
  int sp = s.indexOf(' ');
  if (sp == -1) {
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "Usage: <side> <value>");
    return;
  }
  String side = s.substring(0, sp);
  String val = s.substring(sp + 1);
  side.toUpperCase();
  if (side == "LEFT") {
    if (auto d = abbot::motor::getActiveMotorDriver())
      d->setMotorCommand(abbot::motor::IMotorDriver::MotorSide::LEFT,
                         val.toFloat());
    else
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "No active motor driver");
  } else if (side == "RIGHT") {
    if (auto d = abbot::motor::getActiveMotorDriver())
      d->setMotorCommand(abbot::motor::IMotorDriver::MotorSide::RIGHT,
                         val.toFloat());
    else
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "No active motor driver");
  } else {
    int id = side.toInt();
    if (auto d = abbot::motor::getActiveMotorDriver()) {
      if (id == LEFT_MOTOR_ID)
        d->setMotorCommandRaw(abbot::motor::IMotorDriver::MotorSide::LEFT,
                              (int16_t)val.toInt());
      else if (id == RIGHT_MOTOR_ID)
        d->setMotorCommandRaw(abbot::motor::IMotorDriver::MotorSide::RIGHT,
                              (int16_t)val.toInt());
      else
        LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "Unknown motor id");
    } else
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "No active motor driver");
  }
}

bool MotorCommandHandler::handleMotorGetEncoder(const String &line, const String &up) {
  if (!up.startsWith("MOTOR"))
    return false;
  int p1 = up.indexOf(' ');
  if (p1 == -1)
    return false;
  int p2 = up.indexOf(' ', p1 + 1);
  if (p2 == -1)
    return false;
  String cmd2 = (p2 == -1) ? up.substring(p1 + 1) : up.substring(p1 + 1, p2);
  cmd2.trim();
  if (cmd2 != "GET")
    return false;
  int p3 = up.indexOf(' ', p2 + 1);
  String cmd3 = (p3 == -1) ? up.substring(p2 + 1) : up.substring(p2 + 1, p3);
  cmd3.trim();
  if (cmd3 != "ENCODER")
    return false;
  String arg = (p3 == -1) ? String("") : line.substring(p3 + 1);
  arg.trim();
  abbot::motor::EncoderReport rep;
  bool ok = abbot::motor::getEncoderReportFromArg(arg.c_str(), rep);
  if (!ok) {
    LOG_PRINTLN(abbot::log::CHANNEL_MOTOR,
                "Usage: MOTOR GET ENCODER <LEFT|RIGHT|ID>");
    return true; // Handled but with error
  }
  if (rep.both) {
    LOG_PRINTF(abbot::log::CHANNEL_MOTOR,
               "MOTOR: encoder L(id=%d)=%ld R(id=%d)=%ld\n", rep.leftId,
               (long)rep.leftVal, rep.rightId, (long)rep.rightVal);
  } else {
    if (rep.requestedId == rep.leftId) {
      LOG_PRINTF(abbot::log::CHANNEL_MOTOR,
                 "MOTOR: encoder id=%d value=%ld\n", rep.requestedId,
                 (long)rep.leftVal);
    } else if (rep.requestedId == rep.rightId) {
      LOG_PRINTF(abbot::log::CHANNEL_MOTOR,
                 "MOTOR: encoder id=%d value=%ld\n", rep.requestedId,
                 (long)rep.rightVal);
    } else {
      LOG_PRINTF(abbot::log::CHANNEL_MOTOR,
                 "MOTOR: encoder id=%d value=%ld\n", rep.requestedId,
                 (long)rep.leftVal);
    }
  }
  return true;
}

bool MotorCommandHandler::handleMotorTelemetry(const String &line, const String &up) {
  if (!up.startsWith("MOTOR")) {
    return false;
  }
  int p1 = up.indexOf(' ');
  if (p1 == -1) {
    return false;
  }
  int p2 = up.indexOf(' ', p1 + 1);
  if (p2 == -1) {
    return false;
  }
  String cmd2 = (p2 == -1) ? up.substring(p1 + 1) : up.substring(p1 + 1, p2);
  cmd2.trim();
  if (cmd2 != "TELEMETRY") {
    return false;
  }
  String arg = (p2 == -1) ? String("") : line.substring(p2 + 1);
  arg.trim();
  if (arg.length() == 0) {
    LOG_PRINTLN(abbot::log::CHANNEL_MOTOR,
                "Usage: MOTOR TELEMETRY <LEFT|RIGHT|ALL> <ms> (0 to stop)");
    return true;
  }
  int sp = arg.indexOf(' ');
  if (sp < 0) {
    LOG_PRINTLN(abbot::log::CHANNEL_MOTOR,
                "Usage: MOTOR TELEMETRY <LEFT|RIGHT|ALL> <ms> (0 to stop)");
    return true;
  }
  String sideTok = arg.substring(0, sp);
  String msTok = arg.substring(sp + 1);
  sideTok.trim();
  msTok.trim();
  int ms = msTok.toInt();
  if (ms < 0) {
    LOG_PRINTLN(abbot::log::CHANNEL_MOTOR, "Invalid interval");
    return true;
  }
  bool reportBoth = false;
  bool leftSelected = true;
  String upside = sideTok;
  upside.toUpperCase();
  if (upside == "ALL") {
    reportBoth = true;
  } else if (upside == "LEFT") {
    reportBoth = false;
    leftSelected = true;
  } else if (upside == "RIGHT") {
    reportBoth = false;
    leftSelected = false;
  } else {
    LOG_PRINTLN(abbot::log::CHANNEL_MOTOR,
                "Usage: MOTOR TELEMETRY <LEFT|RIGHT|ALL> <ms> (0 to stop)");
    return true;
  }
  if (ms == 0) {
    g_telemetryManager.stop();
    LOG_PRINTLN(abbot::log::CHANNEL_MOTOR, "MOTOR: telemetry stopped");
    return true;
  }
  g_telemetryManager.stop();
  g_telemetryManager.start(reportBoth, ms, leftSelected);
  LOG_PRINTF(abbot::log::CHANNEL_MOTOR,
             "MOTOR: telemetry started mode=%s interval=%dms\n",
             (reportBoth ? "ALL" : (leftSelected ? "LEFT" : "RIGHT")), ms);
  return true;
}

SerialMenu* MotorCommandHandler::buildMenu() {
  SerialMenu *m = new SerialMenu("Motor Commands");
  m->addEntry(1, "MOTOR ENABLE", [](const String &) {
    auto d = abbot::motor::getActiveMotorDriver();
    if (d) {
      d->enableMotors();
    } else {
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "No active motor driver");
    }
  });
  m->addEntry(2, "MOTOR DISABLE", [](const String &) {
    auto d = abbot::motor::getActiveMotorDriver();
    if (d) {
      d->disableMotors();
    } else {
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "No active motor driver");
    }
  });
  m->addEntry(3, "MOTOR STATUS", [](const String &) {
    auto d = abbot::motor::getActiveMotorDriver();
    if (d) {
      d->printStatus();
    } else {
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "No active motor driver");
    }
  });
  m->addEntry(4, "MOTOR DUMP", [](const String &) {
    auto d = abbot::motor::getActiveMotorDriver();
    if (d) {
      d->dumpConfig();
    } else {
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "No active motor driver");
    }
  });
  m->addEntry(5, "MOTOR RESETPOS", [](const String &) {
    auto d = abbot::motor::getActiveMotorDriver();
    if (d) {
      d->resetPositionTracking();
    } else {
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "No active motor driver");
    }
  });
  m->addEntry(6, "MOTOR POS", [](const String &) {
    auto d = abbot::motor::getActiveMotorDriver();
    if (d) {
      d->processSerialCommand("POS");
    } else {
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "No active motor driver");
    }
  });
  m->addEntry(7, "MOTOR VEL <LEFT|RIGHT> <speed>", [](const String &p) {
    String cmd = "VEL " + p;
    auto d = abbot::motor::getActiveMotorDriver();
    if (d) {
      d->processSerialCommand(cmd);
    } else {
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "No active motor driver");
    }
  });
  m->addEntry(8, "MOTOR SET <LEFT|RIGHT|ID> <v>",
              [this](const String &p) { this->motorSetHandler(p); });
  m->addEntry(9, "MOTOR PARAMS <LEFT|RIGHT>", [](const String &p) {
    String cmd = "PARAMS " + p;
    auto d = abbot::motor::getActiveMotorDriver();
    if (d) {
      d->processSerialCommand(cmd);
    } else {
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "No active motor driver");
    }
  });
  m->addEntry(10, "MOTOR ACC <LEFT|RIGHT> <value>", [](const String &p) {
    String cmd = "ACC " + p;
    auto d = abbot::motor::getActiveMotorDriver();
    if (d) {
      d->processSerialCommand(cmd);
    } else {
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "No active motor driver");
    }
  });
  m->addEntry(11, "MOTOR INVERT <LEFT|RIGHT|ID> [0|1]", [](const String &p) {
    String cmd = "INVERT " + p;
    auto d = abbot::motor::getActiveMotorDriver();
    if (d) {
      d->processSerialCommand(cmd);
    } else {
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "No active motor driver");
    }
  });
  m->addEntry(12, "MOTOR GET ENCODER <LEFT|RIGHT|ID>", [](const String &p) {
    abbot::motor::EncoderReport rep;
    bool ok = abbot::motor::getEncoderReportFromArg(p.c_str(), rep);
    if (!ok) {
      LOG_PRINTLN(abbot::log::CHANNEL_MOTOR,
                  "Usage: MOTOR GET ENCODER <LEFT|RIGHT|ID>");
      return;
    }
    if (rep.both) {
      LOG_PRINTF(abbot::log::CHANNEL_MOTOR,
                 "MOTOR: encoder L(id=%d)=%ld R(id=%d)=%ld\n", rep.leftId,
                 (long)rep.leftVal, rep.rightId, (long)rep.rightVal);
    } else {
      if (rep.requestedId == rep.leftId) {
        LOG_PRINTF(abbot::log::CHANNEL_MOTOR,
                   "MOTOR: encoder id=%d value=%ld\n", rep.requestedId,
                   (long)rep.leftVal);
      } else if (rep.requestedId == rep.rightId) {
        LOG_PRINTF(abbot::log::CHANNEL_MOTOR,
                   "MOTOR: encoder id=%d value=%ld\n", rep.requestedId,
                   (long)rep.rightVal);
      } else {
        LOG_PRINTF(abbot::log::CHANNEL_MOTOR,
                   "MOTOR: encoder id=%d value=%ld\n", rep.requestedId,
                   (long)rep.leftVal);
      }
    }
  });
  m->addEntry(13, "MOTOR SPEED <LEFT|RIGHT|ID>", [](const String &p) {
    abbot::motor::EncoderReport rep;
    bool ok = abbot::motor::getEncoderReportFromArg(p.c_str(), rep);
    if (!ok) {
      LOG_PRINTLN(abbot::log::CHANNEL_MOTOR,
                  "Usage: MOTOR SPEED <LEFT|RIGHT|ID>");
      return;
    }
    auto drv = abbot::motor::getActiveMotorDriver();
    if (!drv) {
      LOG_PRINTLN(abbot::log::CHANNEL_MOTOR, "No active motor driver");
      return;
    }
    if (rep.both) {
      float lsp = drv->readSpeed(abbot::motor::IMotorDriver::MotorSide::LEFT);
      float rsp = drv->readSpeed(abbot::motor::IMotorDriver::MotorSide::RIGHT);
      LOG_PRINTF(abbot::log::CHANNEL_MOTOR,
                 "MOTOR: speed L(id=%d)=%.2f R(id=%d)=%.2f\n", rep.leftId,
                 (double)lsp, rep.rightId, (double)rsp);
    } else {
      if (rep.requestedId == rep.leftId) {
        float lsp = drv->readSpeed(abbot::motor::IMotorDriver::MotorSide::LEFT);
        LOG_PRINTF(abbot::log::CHANNEL_MOTOR,
                   "MOTOR: speed id=%d value=%.2f\n", rep.requestedId,
                   (double)lsp);
      } else if (rep.requestedId == rep.rightId) {
        float rsp = drv->readSpeed(abbot::motor::IMotorDriver::MotorSide::RIGHT);
        LOG_PRINTF(abbot::log::CHANNEL_MOTOR,
                   "MOTOR: speed id=%d value=%.2f\n", rep.requestedId,
                   (double)rsp);
      } else {
        abbot::motor::IMotorDriver::MotorSide side;
        if (abbot::motor::getSideForId(rep.requestedId, side)) {
          float s = abbot::motor::readSpeedBySide(side);
          LOG_PRINTF(abbot::log::CHANNEL_MOTOR,
                     "MOTOR: speed id=%d value=%.2f\n", rep.requestedId,
                     (double)s);
        } else {
          LOG_PRINTF(abbot::log::CHANNEL_MOTOR,
                     "MOTOR: speed id=%d unknown\n", rep.requestedId);
        }
      }
    }
  });
  m->addEntry(14, "MOTOR TELEMETRY <LEFT|RIGHT|ALL> <ms> (0 to stop)", 
              [](const String &p) {
    String args = p;
    args.trim();
    if (args.length() == 0) {
      LOG_PRINTLN(abbot::log::CHANNEL_MOTOR,
                  "Usage: MOTOR TELEMETRY <LEFT|RIGHT|ALL> <ms> (0 to stop)");
      return;
    }
    int spaceIndex = args.indexOf(' ');
    String sideToken;
    String msToken;
    if (spaceIndex < 0) {
      LOG_PRINTLN(abbot::log::CHANNEL_MOTOR,
                  "Usage: MOTOR TELEMETRY <LEFT|RIGHT|ALL> <ms> (0 to stop)");
      return;
    } else {
      sideToken = args.substring(0, spaceIndex);
      msToken = args.substring(spaceIndex + 1);
      sideToken.trim();
      msToken.trim();
    }
    int ms = msToken.toInt();
    if (ms < 0) {
      LOG_PRINTLN(abbot::log::CHANNEL_MOTOR, "Invalid interval");
      return;
    }
    bool reportBoth = false;
    bool leftSelected = true;
    String up = sideToken;
    up.toUpperCase();
    if (up == "ALL") {
      reportBoth = true;
    } else if (up == "LEFT") {
      reportBoth = false;
      leftSelected = true;
    } else if (up == "RIGHT") {
      reportBoth = false;
      leftSelected = false;
    } else {
      LOG_PRINTLN(abbot::log::CHANNEL_MOTOR,
                  "Usage: MOTOR TELEMETRY <LEFT|RIGHT|ALL> <ms> (0 to stop)");
      return;
    }
    if (ms == 0) {
      g_telemetryManager.stop();
      LOG_PRINTLN(abbot::log::CHANNEL_MOTOR, "MOTOR: telemetry stopped");
      return;
    }
    g_telemetryManager.stop();
    g_telemetryManager.start(reportBoth, ms, leftSelected);
    LOG_PRINTF(abbot::log::CHANNEL_MOTOR,
           "MOTOR: telemetry started mode=%s interval=%dms\n",
           (reportBoth ? "ALL" : (leftSelected ? "LEFT" : "RIGHT")), ms);
  });
  return m;
}

} // namespace serialcmds
} // namespace abbot
