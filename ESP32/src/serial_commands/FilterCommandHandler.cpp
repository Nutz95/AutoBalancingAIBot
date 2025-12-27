#include "serial_commands/FilterCommandHandler.h"
#include "logging.h"
#include <Arduino.h>

namespace abbot {
namespace serialcmds {

static constexpr const char *kPrefComplementaryKacc = "c1d_kacc";
static constexpr const char *kPrefComplementaryKbias = "c1d_kbias";
static constexpr const char *kPrefMadgwickBeta = "mad_beta";

FilterCommandHandler::FilterCommandHandler(IFilterService* filterService)
    : m_filterService(filterService) {
    m_menu.reset(new SerialMenu("Filter Selection"));
    
    m_menu->addEntry(1, "FILTER STATUS", [this](const String &) {
        char buf[128];
        snprintf(buf, sizeof(buf), "Current filter: %s",
                 m_filterService->getCurrentFilterName());
        LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, buf);
    });
    m_menu->addEntry(2, "SET ALPHA (usage: FILTER SET ALPHA <v>)", [this](const String &) {
        auto a = m_filterService->getActiveFilter();
        if (a) {
            float val = 0.0f;
            if (a->getParam("ALPHA", val)) {
                char out[128];
                snprintf(out, sizeof(out), "FILTER: ALPHA=%.4f (use: FILTER SET ALPHA <value>)", (double)val);
                LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, out);
            } else {
                LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "FILTER: current filter does not support ALPHA parameter");
            }
        }
    });
    m_menu->addEntry(3, "SET KACC (usage: FILTER SET KACC <v>)", [this](const String &) {
        auto a = m_filterService->getActiveFilter();
        if (a) {
            float val = 0.0f;
            if (a->getParam("KACC", val)) {
                char out[128];
                snprintf(out, sizeof(out), "FILTER: KACC=%.4f (use: FILTER SET KACC <value>)", (double)val);
                LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, out);
            } else {
                LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "FILTER: current filter does not support KACC parameter");
            }
        }
    });
    m_menu->addEntry(4, "SET KBIAS (usage: FILTER SET KBIAS <v>)", [this](const String &) {
        auto a = m_filterService->getActiveFilter();
        if (a) {
            float val = 0.0f;
            if (a->getParam("KBIAS", val)) {
                char out[128];
                snprintf(out, sizeof(out), "FILTER: KBIAS=%.4f (use: FILTER SET KBIAS <value>)", (double)val);
                LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, out);
            } else {
                LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "FILTER: current filter does not support KBIAS parameter");
            }
        }
    });

    // Dynamic filter selection entries
    int count = m_filterService->getAvailableFilterCount();
    int id = 5;
    for (int i = 0; i < count; ++i) {
        const char *name = m_filterService->getAvailableFilterName(i);
        String nameStr = name; // Capture by value
        String label = "SELECT " + nameStr;
        m_menu->addEntry(id++, label.c_str(), [this, nameStr](const String &) {
            this->filterSelectHandler(nameStr);
        });
    }
}

bool FilterCommandHandler::handleCommand(const String& line, const String& up) {
    if (!up.startsWith("FILTER")) {
        return false;
    }
    return handleFilter(line, up);
}

SerialMenu* FilterCommandHandler::buildMenu() {
    return m_menu.get();
}

bool FilterCommandHandler::handleFilter(const String& line, const String& up) {
    String s = up;
    s.trim();
    if (s == "FILTER") {
        LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "FILTER usage: FILTER STATUS | FILTER LIST | FILTER SELECT <name> | FILTER SET <PARAM> <value>");
        return true;
    }

    if (s == "FILTER STATUS") {
        char out[128];
        snprintf(out, sizeof(out), "FILTER: current=%s", m_filterService->getCurrentFilterName());
        LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, out);
        return true;
    } else if (s == "FILTER LIST") {
        int count = m_filterService->getAvailableFilterCount();
        for (int i = 0; i < count; ++i) {
            char out[64];
            snprintf(out, sizeof(out), "FILTER: %s", m_filterService->getAvailableFilterName(i));
            LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, out);
        }
        return true;
    } else if (s.startsWith("FILTER SELECT")) {
        String name = s.substring(14);
        name.trim();
        filterSelectHandler(name);
        return true;
    } else if (s.startsWith("FILTER SET")) {
        filterParamSetHandler(line.substring(11));
        return true;
    }

    return false;
}

void FilterCommandHandler::filterSelectHandler(const String &p) {
    if (m_filterService->selectFilter(p.c_str())) {
        m_filterService->applyParamsFromPrefs();
        auto a = m_filterService->getActiveFilter();
        if (a) {
            unsigned long ms = a->getWarmupDurationMs();
            if (ms > 0) {
                float s = ((float)ms) / 1000.0f;
                m_filterService->requestWarmup(s);
                char tbuf[128];
                snprintf(tbuf, sizeof(tbuf), "FILTER: requested warmup %.3f s for %s", (double)s, p.c_str());
                LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, tbuf);
            }
        }
    } else {
        LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "FILTER: unknown filter");
    }
}

void FilterCommandHandler::filterParamSetHandler(const String &p) {
    String s = p;
    s.trim();
    int sp = s.indexOf(' ');
    if (sp == -1) {
        LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "Usage: FILTER SET <PARAM> <value>");
        return;
    }
    String param = s.substring(0, sp);
    param.toUpperCase();
    float v = s.substring(sp + 1).toFloat();
    
    auto a = m_filterService->getActiveFilter();
    if (!a) {
        LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "FILTER: no active filter");
        return;
    }

    const char *pname = nullptr;
    if (param == "ALPHA") pname = "ALPHA";
    else if (param == "KACC") pname = "KACC";
    else if (param == "KBIAS") pname = "KBIAS";
    else if (param == "BETA") pname = "BETA";

    if (!pname) {
        LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "FILTER SET usage: FILTER SET <ALPHA|KACC|KBIAS|BETA> <value>");
        return;
    }

    if (a->setParam(pname, v)) {
        char out[160];
        snprintf(out, sizeof(out), "FILTER: %s set to %.6f", pname, (double)v);
        LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, out);
        
        if (strcmp(pname, "KACC") == 0) m_filterService->saveParamToPrefs(kPrefComplementaryKacc, v);
        else if (strcmp(pname, "KBIAS") == 0) m_filterService->saveParamToPrefs(kPrefComplementaryKbias, v);
        else if (strcmp(pname, "BETA") == 0) m_filterService->saveParamToPrefs(kPrefMadgwickBeta, v);
    } else {
        LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "FILTER: active filter does not support parameter or value out of range");
    }
}

} // namespace serialcmds
} // namespace abbot
