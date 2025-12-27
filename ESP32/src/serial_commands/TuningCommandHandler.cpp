#include "serial_commands/TuningCommandHandler.h"
#include "tuning_capture.h"
#include "logging.h"
#include <Arduino.h>

namespace abbot {
namespace serialcmds {

static void pushTuningMask() {
  abbot::log::pushChannelMask(
      static_cast<uint32_t>(abbot::log::CHANNEL_DEFAULT) |
      static_cast<uint32_t>(abbot::log::CHANNEL_TUNING));
}

static void startTuningStream() {
  pushTuningMask();
  abbot::log::enableChannel(abbot::log::CHANNEL_TUNING);
  LOG_PRINTLN(abbot::log::CHANNEL_TUNING,
              "timestamp_ms,pitch_deg,pitch_rad,pitch_rate_deg,pitch_rate_rad,"
              "ax,ay,az,gx,gy,gz,temp_C,left_cmd,right_cmd");
  LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "TUNING: started");
}

static void stopTuningStream() {
  abbot::log::disableChannel(abbot::log::CHANNEL_TUNING);
  abbot::log::popChannelMask();
  LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "TUNING: stopped");
}

static void startTuningCapture(uint32_t samples, bool csv, bool statsOnly = false) {
  pushTuningMask();
  if (statsOnly) {
    abbot::tuning::startCapture(samples, false, true);
  } else {
    abbot::tuning::startCapture(samples, csv);
  }
}

TuningCommandHandler::TuningCommandHandler() {
    m_menu.reset(new SerialMenu("Madgwick tuning"));
    m_menu->addEntry(1, "TUNING START (stream)", [this](const String &p) { this->tuningStreamStartHandler(p); });
    m_menu->addEntry(2, "TUNING STOP", [this](const String &p) { this->tuningStreamStopHandler(p); });
    m_menu->addEntry(3, "TUNING CAPTURE 1000", [](const String &) { startTuningCapture(1000, true); });
    m_menu->addEntry(4, "TUNING CAPTURE 2000", [](const String &) { startTuningCapture(2000, true); });
    m_menu->addEntry(5, "TUNING CAPTURE 5000", [](const String &) { startTuningCapture(5000, true); });
    m_menu->addEntry(6, "TUNING CAPTURE CUSTOM", [this](const String &p) { this->tuningCustomHandler(p); });
}

bool TuningCommandHandler::handleCommand(const String& line, const String& up) {
    if (!up.startsWith("TUNING")) {
        return false;
    }
    return handleTuning(line, up);
}

SerialMenu* TuningCommandHandler::buildMenu() {
    return m_menu.get();
}

SerialMenu* TuningCommandHandler::getMenu() {
    return m_menu.get();
}

bool TuningCommandHandler::handleTuning(const String& line, const String& up) {
    String s = up;
    s.trim();
    if (s == "TUNING") {
        LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "TUNING usage: TUNING START [N] | TUNING STOP | TUNING START_STATS [N]");
        return true;
    }

    if (s.startsWith("TUNING START_STATS")) {
        int n = 2000;
        if (s.length() > 18) n = s.substring(18).toInt();
        if (n <= 0) n = 2000;
        startTuningCapture((uint32_t)n, false, true);
        LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "TUNING: auto-capture (stats-only) started");
        return true;
    } else if (s.startsWith("TUNING START")) {
        String arg = s.substring(12);
        arg.trim();
        if (arg.length() > 0) {
            int n = arg.toInt();
            if (n > 0) {
                startTuningCapture((uint32_t)n, true);
                LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "TUNING: auto-capture started");
                return true;
            }
        }
        startTuningStream();
        return true;
    } else if (s == "TUNING STOP") {
        stopTuningStream();
        abbot::tuning::stopCapture();
        return true;
    }

    return false;
}

void TuningCommandHandler::tuningCustomHandler(const String &p) {
  String s = p;
  s.trim();
  if (s.length() == 0) {
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "Usage: <N> (number of samples)");
    return;
  }
  int n = s.toInt();
  if (n <= 0) {
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "Invalid sample count");
    return;
  }
  startTuningCapture((uint32_t)n, true);
}

void TuningCommandHandler::tuningStreamStartHandler(const String &) {
  startTuningStream();
}

void TuningCommandHandler::tuningStreamStopHandler(const String &) {
  stopTuningStream();
}

} // namespace serialcmds
} // namespace abbot
