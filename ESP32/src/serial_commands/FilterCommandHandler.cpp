#include "serial_commands/FilterCommandHandler.h"
#include "filter_manager.h"
#include "logging.h"
#include "SystemTasks.h"
#include <Arduino.h>
#include <Preferences.h>

namespace abbot {
namespace serialcmds {

static constexpr const char *kPrefComplementaryKacc = "c1d_kacc";
static constexpr const char *kPrefComplementaryKbias = "c1d_kbias";
static constexpr const char *kPrefMadgwickBeta = "mad_beta";

static void applyComplementaryParamsFromPrefs() {
  Preferences pref;
  if (!pref.begin("abbot", true)) {
    return;
  }
  auto f = abbot::filter::getActiveFilter();
  const char *fname = abbot::filter::getCurrentFilterName();
  if (!f || !fname || strcmp(fname, "COMPLEMENTARY1D") != 0) {
    pref.end();
    return;
  }
  bool applied = false;
  float val = 0.0f;
  if (pref.isKey(kPrefComplementaryKacc)) {
    val = pref.getFloat(kPrefComplementaryKacc, 0.02f);
    f->setParam("KACC", val);
    applied = true;
  }
  if (pref.isKey(kPrefComplementaryKbias)) {
    val = pref.getFloat(kPrefComplementaryKbias, 0.01f);
    f->setParam("KBIAS", val);
    applied = true;
  }
  if (applied) {
    float kacc = pref.getFloat(kPrefComplementaryKacc, 0.0f);
    float kbias = pref.getFloat(kPrefComplementaryKbias, 0.0f);
    char msg[180];
    snprintf(msg, sizeof(msg),
             "FILTER: restored complementary params from NVS (KACC=%.4f KBIAS=%.4f)",
             (double)kacc, (double)kbias);
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, msg);
  }
  pref.end();
}

static void applyMadgwickParamFromPrefs() {
  Preferences pref;
  if (!pref.begin("abbot", true)) {
    return;
  }
  auto f = abbot::filter::getActiveFilter();
  const char *fname = abbot::filter::getCurrentFilterName();
  if (!f || !fname || strcmp(fname, "MADGWICK") != 0) {
    pref.end();
    return;
  }
  bool applied = false;
  float val = 0.0f;
  if (pref.isKey(kPrefMadgwickBeta)) {
    val = pref.getFloat(kPrefMadgwickBeta, 0.021909f);
    f->setParam("BETA", val);
    applied = true;
  }
  if (applied) {
    float beta = pref.getFloat(kPrefMadgwickBeta, 0.0f);
    char msg[128];
    snprintf(msg, sizeof(msg), "FILTER: restored Madgwick beta from NVS (BETA=%.8f)", (double)beta);
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, msg);
  }
  pref.end();
}

FilterCommandHandler::FilterCommandHandler() {
    m_menu = new SerialMenu("Filter Selection");
    m_menu->setOnEnter([this]() {
        m_menu->clearEntries();
        m_menu->addEntry(1, "FILTER STATUS", [](const String &) {
            char buf[128];
            snprintf(buf, sizeof(buf), "Current filter: %s",
                     abbot::filter::getCurrentFilterName());
            LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, buf);
        });
        m_menu->addEntry(2, "SET ALPHA (usage: FILTER SET ALPHA <v>)", [](const String &) {
            auto a = abbot::filter::getActiveFilter();
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
        m_menu->addEntry(3, "SET KACC (usage: FILTER SET KACC <v>)", [](const String &) {
            auto a = abbot::filter::getActiveFilter();
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
        m_menu->addEntry(4, "SET KBIAS (usage: FILTER SET KBIAS <v>)", [](const String &) {
            auto a = abbot::filter::getActiveFilter();
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

        int count = abbot::filter::getAvailableFilterCount();
        int id = 5;
        const char *cur = abbot::filter::getCurrentFilterName();
        for (int i = 0; i < count; ++i) {
            const char *name = abbot::filter::getAvailableFilterName(i);
            char lbl[80];
            if (cur && strcmp(name, cur) == 0) {
                snprintf(lbl, sizeof(lbl), "SELECT %s (current)", name);
                m_menu->addEntry(id++, lbl, [name](const String &) {
                    char b[128];
                    snprintf(b, sizeof(b), "FILTER: %s already active", name);
                    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, b);
                });
            } else {
                snprintf(lbl, sizeof(lbl), "SELECT %s", name);
                m_menu->addEntry(id++, lbl, [name](const String &) {
                    filterSelectHandler(name);
                });
            }
        }
    });
}

bool FilterCommandHandler::handleCommand(const String& line, const String& up) {
    if (!up.startsWith("FILTER")) {
        return false;
    }
    return handleFilter(line, up);
}

SerialMenu* FilterCommandHandler::buildMenu() {
    return m_menu;
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
        snprintf(out, sizeof(out), "FILTER: current=%s", abbot::filter::getCurrentFilterName());
        LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, out);
        return true;
    } else if (s == "FILTER LIST") {
        int count = abbot::filter::getAvailableFilterCount();
        for (int i = 0; i < count; ++i) {
            char out[64];
            snprintf(out, sizeof(out), "FILTER: %s", abbot::filter::getAvailableFilterName(i));
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
    const char *name = p.c_str();
    bool ok = abbot::filter::setCurrentFilterByName(name);
    if (!ok) {
        LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "FILTER: unknown filter");
    } else {
        applyComplementaryParamsFromPrefs();
        applyMadgwickParamFromPrefs();
        auto a = abbot::filter::getActiveFilter();
        if (a) {
            unsigned long ms = a->getWarmupDurationMs();
            if (ms > 0) {
                float s = ((float)ms) / 1000.0f;
                requestTuningWarmupSeconds(s);
                char tbuf[128];
                snprintf(tbuf, sizeof(tbuf), "FILTER: requested warmup %.3f s for %s", (double)s, name);
                LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, tbuf);
            }
        }
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
    
    auto a = abbot::filter::getActiveFilter();
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
        Preferences pref;
        if (pref.begin("abbot", false)) {
            if (strcmp(pname, "KACC") == 0) pref.putFloat(kPrefComplementaryKacc, v);
            else if (strcmp(pname, "KBIAS") == 0) pref.putFloat(kPrefComplementaryKbias, v);
            else if (strcmp(pname, "BETA") == 0) pref.putFloat(kPrefMadgwickBeta, v);
            pref.end();
        }
    } else {
        LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "FILTER: active filter does not support parameter or value out of range");
    }
}

} // namespace serialcmds
} // namespace abbot
