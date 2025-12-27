#include "serial_commands/AutotuneCommandHandler.h"
#include "balancer_controller.h"
#include "logging.h"
#include <Arduino.h>

namespace abbot {
namespace serialcmds {

AutotuneCommandHandler::AutotuneCommandHandler() {
    m_menu = new SerialMenu("PID Autotune");
    m_menu->addEntry(1, "AUTOTUNE START", [](const String &) {
      abbot::balancer::controller::startAutotune();
    });
    m_menu->addEntry(2, "AUTOTUNE STOP", [](const String &) {
      abbot::balancer::controller::stopAutotune();
    });
    m_menu->addEntry(3, "AUTOTUNE STATUS", [](const String &) {
      const char *status = abbot::balancer::controller::getAutotuneStatus();
      char buf[128];
      snprintf(buf, sizeof(buf), "AUTOTUNE: %s", status);
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, buf);
    });
    m_menu->addEntry(4, "AUTOTUNE APPLY", [](const String &) {
      abbot::balancer::controller::applyAutotuneGains();
    });
    m_menu->addEntry(5, "AUTOTUNE RELAY <amp>",
                  [](const String &p) { autotuneRelayHandler(p); });
    m_menu->addEntry(6, "AUTOTUNE DEADBAND <deg>",
                  [](const String &p) { autotuneDeadbandHandler(p); });
    m_menu->addEntry(7, "AUTOTUNE MAXANGLE <deg>",
                  [](const String &p) { autotuneMaxAngleHandler(p); });
}

bool AutotuneCommandHandler::handleCommand(const String& line, const String& up) {
    if (!up.startsWith("AUTOTUNE")) {
        return false;
    }

    String s = up;
    s.trim();
    if (s == "AUTOTUNE") {
        LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT,
                    "AUTOTUNE usage: AUTOTUNE START | AUTOTUNE STOP | AUTOTUNE "
                    "STATUS | AUTOTUNE APPLY | AUTOTUNE RELAY <amp> | AUTOTUNE "
                    "DEADBAND <deg> | AUTOTUNE MAXANGLE <deg>");
        return true;
    }

    if (s == "AUTOTUNE START") {
        abbot::balancer::controller::startAutotune();
    } else if (s == "AUTOTUNE SAFE") {
        abbot::balancer::controller::setAutotuneRelay(0.15f);
        abbot::balancer::controller::setAutotuneDeadband(0.5f);
        abbot::balancer::controller::setAutotuneMaxAngle(8.0f);
        LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT,
                    "AUTOTUNE: starting in SAFE mode (relay=0.15 deadband=0.5° "
                    "maxangle=8°)");
        abbot::balancer::controller::startAutotune();
    } else if (s == "AUTOTUNE STOP") {
        abbot::balancer::controller::stopAutotune();
    } else if (s == "AUTOTUNE STATUS") {
        const char *status = abbot::balancer::controller::getAutotuneStatus();
        char buf[128];
        snprintf(buf, sizeof(buf), "AUTOTUNE: %s", status);
        LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, buf);
    } else if (s == "AUTOTUNE APPLY") {
        abbot::balancer::controller::applyAutotuneGains();
    } else if (s.startsWith("AUTOTUNE RELAY")) {
        autotuneRelayHandler(line.substring(14));
    } else if (s.startsWith("AUTOTUNE DEADBAND")) {
        autotuneDeadbandHandler(line.substring(17));
    } else if (s.startsWith("AUTOTUNE MAXANGLE")) {
        autotuneMaxAngleHandler(line.substring(17));
    } else {
        return false;
    }
    return true;
}

SerialMenu* AutotuneCommandHandler::buildMenu() {
    return m_menu;
}

void AutotuneCommandHandler::autotuneRelayHandler(const String &p) {
  String s = p;
  s.trim();
  if (s.length() == 0) {
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT,
                "Usage: AUTOTUNE RELAY <amplitude>");
    return;
  }
  float amp = s.toFloat();
  abbot::balancer::controller::setAutotuneRelay(amp);
}

void AutotuneCommandHandler::autotuneDeadbandHandler(const String &p) {
  String s = p;
  s.trim();
  if (s.length() == 0) {
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT,
                "Usage: AUTOTUNE DEADBAND <degrees>");
    return;
  }
  float db = s.toFloat();
  abbot::balancer::controller::setAutotuneDeadband(db);
}

void AutotuneCommandHandler::autotuneMaxAngleHandler(const String &p) {
  String s = p;
  s.trim();
  if (s.length() == 0) {
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT,
                "Usage: AUTOTUNE MAXANGLE <degrees>");
    return;
  }
  float maxa = s.toFloat();
  abbot::balancer::controller::setAutotuneMaxAngle(maxa);
}

} // namespace serialcmds
} // namespace abbot
