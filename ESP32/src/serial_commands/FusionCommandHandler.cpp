#include "serial_commands/FusionCommandHandler.h"
#include "logging.h"
#include <Arduino.h>

namespace abbot {
namespace serialcmds {

FusionCommandHandler::FusionCommandHandler(IFusionService* service)
    : m_service(service) {}

bool FusionCommandHandler::handleCommand(const String& line, const String& up) {
    if (!up.startsWith("FUSION")) {
        return false;
    }

    String s = up;
    s.trim();
    if (s == "FUSION") {
        return true;
    }

    if (s == "FUSION STATUS") {
        if (m_service) {
            m_service->printDiagnostics();
            bool ready = m_service->isReady();
            unsigned long rem = m_service->getWarmupRemaining();
            char out[128];
            snprintf(out, sizeof(out), "FUSION: ready=%d warmup_remaining=%lu",
                     ready ? 1 : 0, rem);
            LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, out);
        }
        return true;
    } else if (s.startsWith("FUSION WARMUP")) {
        String arg = s.substring(13);
        arg.trim();
        if (arg.length() == 0) {
            LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "Usage: FUSION WARMUP <seconds>");
            return true;
        }
        float secs = arg.toFloat();
        if (secs <= 0.0f) {
            LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "Invalid warmup seconds");
            return true;
        }
        if (m_service) {
            m_service->requestWarmup(secs);
            char out[128];
            snprintf(out, sizeof(out), "FUSION: warmup requested ~%.1f s", (double)secs);
            LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, out);
        }
        return true;
    }

    return false;
}

SerialMenu* FusionCommandHandler::buildMenu() {
    return nullptr;
}

} // namespace serialcmds
} // namespace abbot
