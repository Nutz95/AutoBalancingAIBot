#include "serial_commands/LogCommandHandler.h"
#include "logging.h"
#include <Arduino.h>

namespace abbot {
namespace serialcmds {

LogCommandHandler::LogCommandHandler() {
    m_menu = new SerialMenu("Log channels");
    m_menu->setOnEnter([this]() { logMenuOnEnter(); });
}

bool LogCommandHandler::handleCommand(const String& line, const String& up) {
    if (!up.startsWith("LOG")) {
        return false;
    }
    return handleLog(line, up);
}

SerialMenu* LogCommandHandler::buildMenu() {
    return m_menu;
}

bool LogCommandHandler::handleLog(const String& line, const String& up) {
    String s = up;
    s.trim();
    if (s == "LOG") {
        LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "LOG usage: LOG LIST | LOG ENABLE <channel> | LOG DISABLE <channel>");
        return true;
    }

    if (s == "LOG LIST") {
        char out[128];
        abbot::log::listEnabledChannels(out, sizeof(out));
        LOG_PRINTF(abbot::log::CHANNEL_DEFAULT, "LOG enabled: %s\n", out);
        return true;
    }

    // Simple parsing for ENABLE/DISABLE
    if (s.startsWith("LOG ENABLE") || s.startsWith("LOG DISABLE")) {
        bool enable = s.startsWith("LOG ENABLE");
        String argch = s.substring(enable ? 11 : 12);
        argch.trim();
        if (argch.length() == 0) {
            LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "Usage: LOG ENABLE|DISABLE <channel>");
            return true;
        }
        auto ch = abbot::log::channelFromString(argch.c_str());
        if (ch == static_cast<abbot::log::Channel>(0)) {
            LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "Unknown channel. Known: TUNING,BLE,IMU,MOTOR,DEFAULT");
            return true;
        }
        if (enable) {
            abbot::log::enableChannel(ch);
            LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "LOG: enabled");
        } else {
            abbot::log::disableChannel(ch);
            LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "LOG: disabled");
        }
        return true;
    }

    return false;
}

void LogCommandHandler::logMenuOnEnter() {
    m_menu->clearEntries();
    struct C {
        abbot::log::Channel ch;
        const char *name;
    } channels[] = {
        {abbot::log::CHANNEL_TUNING, "TUNING"},
        {abbot::log::CHANNEL_BLE, "BLE"},
        {abbot::log::CHANNEL_IMU, "IMU"},
        {abbot::log::CHANNEL_MOTOR, "MOTOR"},
        {abbot::log::CHANNEL_BALANCER, "BALANCER"},
        {abbot::log::CHANNEL_DEFAULT, "DEFAULT"},
    };
    int id = 1;
    for (auto &c : channels) {
        const char *name = abbot::log::channelName(c.ch);
        bool on = abbot::log::isChannelEnabled(c.ch);
        char lbl[64];
        snprintf(lbl, sizeof(lbl), "%s: %s", name, on ? "ON" : "OFF");
        abbot::log::Channel chcopy = c.ch;
        m_menu->addEntry(id++, lbl, [this, chcopy](const String &) {
            bool newState = abbot::log::toggleChannel(chcopy);
            char msg[64];
            snprintf(msg, sizeof(msg), "%s %s", abbot::log::channelName(chcopy),
                     newState ? "enabled" : "disabled");
            LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, msg);
            m_menu->enter();
        });
    }
}

} // namespace serialcmds
} // namespace abbot
