#include "serial_commands/MotorCommandHandler.h"
#include "motor_drivers/driver_manager.h"
#include "logging.h"
#include "../../config/motor_configs/mks_servo_config.h" // Prefer MKS IDs if present
#include "../../config/motor_configs/servo_motor_config.h" // Fallback for other motors

namespace abbot {
namespace serialcmds {

MotorCommandHandler::MotorCommandHandler(IMotorService* motorService)
    : m_motorService(motorService) {
    m_menu.reset(new SerialMenu("Motor Commands"));
    
    m_menu->addEntry(1, "MOTOR ENABLE", [this](const String &) {
        if (auto driver = m_motorService->getActiveDriver()) {
            driver->enableMotors();
        } else {
            LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "No active motor driver");
        }
    });
    m_menu->addEntry(2, "MOTOR DISABLE", [this](const String &) {
        if (auto driver = m_motorService->getActiveDriver()) {
            driver->disableMotors();
        } else {
            LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "No active motor driver");
        }
    });
    m_menu->addEntry(3, "MOTOR STATUS", [this](const String &) {
        if (auto driver = m_motorService->getActiveDriver()) {
            driver->printStatus();
        } else {
            LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "No active motor driver");
        }
    });
    m_menu->addEntry(4, "MOTOR DUMP", [this](const String &) {
        if (auto driver = m_motorService->getActiveDriver()) {
            driver->dumpConfig();
        } else {
            LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "No active motor driver");
        }
    });
    m_menu->addEntry(5, "MOTOR RESETPOS", [this](const String &) {
        if (auto driver = m_motorService->getActiveDriver()) {
            driver->resetPositionTracking();
        } else {
            LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "No active motor driver");
        }
    });
    m_menu->addEntry(6, "MOTOR POS", [this](const String &) {
        if (auto driver = m_motorService->getActiveDriver()) {
            driver->processSerialCommand("POS");
        } else {
            LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "No active motor driver");
        }
    });
    m_menu->addEntry(7, "MOTOR VEL <LEFT|RIGHT> <speed>", [this](const String &p) {
        String cmd = "VEL " + p;
        if (auto driver = m_motorService->getActiveDriver()) {
            driver->processSerialCommand(cmd);
        } else {
            LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "No active motor driver");
        }
    });
    m_menu->addEntry(8, "MOTOR SET <LEFT|RIGHT|BOTH|ID> <v> [v2]",
                [this](const String &p) { this->motorSetHandler(p); });
    m_menu->addEntry(9, "MOTOR PARAMS <LEFT|RIGHT>", [this](const String &p) {
        String cmd = "PARAMS " + p;
        if (auto driver = m_motorService->getActiveDriver()) {
            driver->processSerialCommand(cmd);
        } else {
            LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "No active motor driver");
        }
    });
    m_menu->addEntry(10, "MOTOR ACC <LEFT|RIGHT> <value>", [this](const String &p) {
        String cmd = "ACC " + p;
        if (auto driver = m_motorService->getActiveDriver()) {
            driver->processSerialCommand(cmd);
        } else {
            LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "No active motor driver");
        }
    });
    m_menu->addEntry(11, "MOTOR INVERT <LEFT|RIGHT|ID> [0|1]", [this](const String &p) {
        String cmd = "INVERT " + p;
        if (auto driver = m_motorService->getActiveDriver()) {
            driver->processSerialCommand(cmd);
        } else {
            LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "No active motor driver");
        }
    });
    m_menu->addEntry(12, "MOTOR GET ENCODER <LEFT|RIGHT|ID>", [this](const String &p) {
        abbot::motor::EncoderReport rep;
        bool ok = abbot::motor::getEncoderReportFromArg(p.c_str(), rep);
        if (!ok) {
            LOG_PRINTLN(abbot::log::CHANNEL_MOTOR, "Usage: MOTOR GET ENCODER <LEFT|RIGHT|ID>");
            return;
        }
        if (rep.both) {
            LOG_PRINTF(abbot::log::CHANNEL_MOTOR, "MOTOR: encoder L(id=%d)=%ld R(id=%d)=%ld\n", 
                       rep.leftId, (long)rep.leftVal, rep.rightId, (long)rep.rightVal);
        } else {
            long val = (rep.requestedId == rep.rightId) ? (long)rep.rightVal : (long)rep.leftVal;
            LOG_PRINTF(abbot::log::CHANNEL_MOTOR, "MOTOR: encoder id=%d value=%ld\n", rep.requestedId, val);
        }
    });
    m_menu->addEntry(13, "MOTOR SPEED <LEFT|RIGHT|ID>", [this](const String &p) {
        abbot::motor::EncoderReport rep;
        bool ok = abbot::motor::getEncoderReportFromArg(p.c_str(), rep);
        if (!ok) {
            LOG_PRINTLN(abbot::log::CHANNEL_MOTOR, "Usage: MOTOR SPEED <LEFT|RIGHT|ID>");
            return;
        }
        auto drv = m_motorService->getActiveDriver();
        if (!drv) {
            LOG_PRINTLN(abbot::log::CHANNEL_MOTOR, "No active motor driver");
            return;
        }
        if (rep.both) {
            float lsp = drv->readSpeed(abbot::motor::IMotorDriver::MotorSide::LEFT);
            float rsp = drv->readSpeed(abbot::motor::IMotorDriver::MotorSide::RIGHT);
            LOG_PRINTF(abbot::log::CHANNEL_MOTOR, "MOTOR: speed L(id=%d)=%.2f R(id=%d)=%.2f\n", 
                       rep.leftId, (double)lsp, rep.rightId, (double)rsp);
        } else {
            float s = 0;
            if (rep.requestedId == rep.leftId) {
                s = drv->readSpeed(abbot::motor::IMotorDriver::MotorSide::LEFT);
            } else if (rep.requestedId == rep.rightId) {
                s = drv->readSpeed(abbot::motor::IMotorDriver::MotorSide::RIGHT);
            } else {
                abbot::motor::IMotorDriver::MotorSide side;
                if (abbot::motor::getSideForId(rep.requestedId, side)) {
                    s = drv->readSpeed(side);
                } else {
                    LOG_PRINTF(abbot::log::CHANNEL_MOTOR, "MOTOR: speed id=%d unknown\n", rep.requestedId);
                    return;
                }
            }
            LOG_PRINTF(abbot::log::CHANNEL_MOTOR, "MOTOR: speed id=%d value=%.2f\n", rep.requestedId, (double)s);
        }
    });
    m_menu->addEntry(14, "MOTOR TELEMETRY <LEFT|RIGHT|ALL> <ms> (0 to stop)", [this](const String &p) {
        this->handleMotorTelemetry("MOTOR TELEMETRY " + p, "MOTOR TELEMETRY " + p);
    });
    m_menu->addEntry(15, "MOTOR SCAN", [this](const String &) {
        if (auto driver = m_motorService->getActiveDriver()) {
            driver->processSerialCommand("SCAN");
        } else {
            LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "No active motor driver");
        }
    });
    m_menu->addEntry(16, "MOTOR READ ALL", [this](const String &) {
        if (auto driver = m_motorService->getActiveDriver()) {
            driver->processSerialCommand("READ ALL");
        } else {
            LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "No active motor driver");
        }
    });
}

bool MotorCommandHandler::handleCommand(const String& line, const String& lineUpper) {
    if (!lineUpper.startsWith("MOTOR")) {
        return false;
    }

    if (handleMotorGetEncoder(line, lineUpper)) {
        return true;
    }
    if (handleMotorTelemetry(line, lineUpper)) {
        return true;
    }

    // Fallback for other MOTOR commands that might be handled by the driver directly
    if (auto driver = m_motorService->getActiveDriver()) {
        int spaceIdx = lineUpper.indexOf(' ');
        if (spaceIdx != -1) {
            String subCmd = line.substring(spaceIdx + 1);
            subCmd.trim();
            if (driver->processSerialCommand(subCmd)) {
                return true;
            }
        }
    }

    return false;
}

SerialMenu* MotorCommandHandler::buildMenu() {
    return m_menu.get();
}

void MotorCommandHandler::motorSetHandler(const String &p) {
  String s = p;
  s.trim();
  int sp = s.indexOf(' ');
  if (sp == -1) {
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "Usage: <side|BOTH> <value> [value2]");
    return;
  }
  String side = s.substring(0, sp);
  String valStr = s.substring(sp + 1);
  valStr.trim();
  side.toUpperCase();
  
  auto driver = m_motorService->getActiveDriver();
  if (!driver) {
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "No active motor driver");
    return;
  }

  if (side == "LEFT") {
    driver->setMotorCommand(abbot::motor::IMotorDriver::MotorSide::LEFT, valStr.toFloat());
  } else if (side == "RIGHT") {
    driver->setMotorCommand(abbot::motor::IMotorDriver::MotorSide::RIGHT, valStr.toFloat());
  } else if (side == "BOTH") {
      int sp2 = valStr.indexOf(' ');
      if (sp2 != -1) {
          float vL = valStr.substring(0, sp2).toFloat();
          float vR = valStr.substring(sp2 + 1).toFloat();
          driver->setMotorCommandBoth(vL, vR);
          LOG_PRINTF(abbot::log::CHANNEL_DEFAULT, "MOTOR: set BOTH L=%.3f R=%.3f\n", (double)vL, (double)vR);
      } else {
          float v = valStr.toFloat();
          driver->setMotorCommandBoth(v, v);
          LOG_PRINTF(abbot::log::CHANNEL_DEFAULT, "MOTOR: set BOTH L=%.3f R=%.3f\n", (double)v, (double)v);
      }
  } else {
    int id = side.toInt();
    if (id == (int)LEFT_MOTOR_ID) {
      driver->setMotorCommandRaw(abbot::motor::IMotorDriver::MotorSide::LEFT, (int16_t)valStr.toInt());
    } else if (id == (int)RIGHT_MOTOR_ID) {
      driver->setMotorCommandRaw(abbot::motor::IMotorDriver::MotorSide::RIGHT, (int16_t)valStr.toInt());
    } else if (id == 0) {
      // maybe it was "BOTH" but lower case? Already handled by toUpperCase
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "Unknown motor ID or side. Use LEFT, RIGHT, BOTH or ID.");
    } else {
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "Unknown motor side or ID");
    }
  }
}

bool MotorCommandHandler::handleMotorGetEncoder(const String &line, const String &up) {
  if (!up.startsWith("MOTOR GET ENCODER")) {
    return false;
  }
  
  String arg = line.substring(17);
  arg.trim();
  
  abbot::motor::EncoderReport rep;
  if (!abbot::motor::getEncoderReportFromArg(arg.c_str(), rep)) {
    LOG_PRINTLN(abbot::log::CHANNEL_MOTOR, "Usage: MOTOR GET ENCODER <LEFT|RIGHT|ID>");
    return true;
  }
  
  if (rep.both) {
    LOG_PRINTF(abbot::log::CHANNEL_MOTOR, "MOTOR: encoder L(id=%d)=%ld R(id=%d)=%ld\n", 
               rep.leftId, (long)rep.leftVal, rep.rightId, (long)rep.rightVal);
  } else {
    long val = (rep.requestedId == rep.rightId) ? (long)rep.rightVal : (long)rep.leftVal;
    LOG_PRINTF(abbot::log::CHANNEL_MOTOR, "MOTOR: encoder id=%d value=%ld\n", rep.requestedId, val);
  }
  return true;
}

bool MotorCommandHandler::handleMotorTelemetry(const String &line, const String &up) {
  if (!up.startsWith("MOTOR TELEMETRY")) {
    return false;
  }
  
  String arg = line.substring(15);
  arg.trim();
    if (arg.length() == 0) {
    LOG_PRINTLN(abbot::log::CHANNEL_MOTOR, "Usage: MOTOR TELEMETRY <LEFT|RIGHT|ALL> <ms> (0 to stop)");
    return true;
  }

    auto activeDriver = m_motorService->getActiveDriver();
    if (activeDriver) {
        String driverCmd = "TELEMETRY " + arg;
        activeDriver->processSerialCommand(driverCmd);
    }
  
  int sp = arg.indexOf(' ');
  if (sp < 0) {
    LOG_PRINTLN(abbot::log::CHANNEL_MOTOR, "Usage: MOTOR TELEMETRY <LEFT|RIGHT|ALL> <ms> (0 to stop)");
    return true;
  }
  
  String sideTok = arg.substring(0, sp);
  int ms = arg.substring(sp + 1).toInt();
  if (ms < 0) {
    LOG_PRINTLN(abbot::log::CHANNEL_MOTOR, "Invalid interval");
    return true;
  }
  
  if (ms == 0) {
    m_motorService->stopTelemetry();
    LOG_PRINTLN(abbot::log::CHANNEL_MOTOR, "MOTOR: telemetry stopped");
    return true;
  }

  bool reportBoth = false;
  bool leftSelected = true;
  sideTok.toUpperCase();
  if (sideTok == "ALL") {
    reportBoth = true;
  } else if (sideTok == "LEFT") {
    reportBoth = false;
    leftSelected = true;
  } else if (sideTok == "RIGHT") {
    reportBoth = false;
    leftSelected = false;
  } else {
    LOG_PRINTLN(abbot::log::CHANNEL_MOTOR, "Usage: MOTOR TELEMETRY <LEFT|RIGHT|ALL> <ms> (0 to stop)");
    return true;
  }
  
  m_motorService->startTelemetry(reportBoth, ms, leftSelected);
  LOG_PRINTF(abbot::log::CHANNEL_MOTOR, "MOTOR: telemetry started mode=%s interval=%dms\n",
             (reportBoth ? "ALL" : (leftSelected ? "LEFT" : "RIGHT")), ms);
  return true;
}

} // namespace serialcmds
} // namespace abbot
