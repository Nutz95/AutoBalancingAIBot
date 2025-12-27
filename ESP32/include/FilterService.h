#ifndef FILTER_SERVICE_H
#define FILTER_SERVICE_H

#include "IFilterService.h"
#include "filter_manager.h"
#include "logging.h"
#include "SystemTasks.h"
#include <Preferences.h>

namespace abbot {

class FilterService : public IFilterService {
public:
    abbot::IMUFilter* getActiveFilter() override {
        return abbot::filter::getActiveFilter().get();
    }

    const char* getCurrentFilterName() override {
        return abbot::filter::getCurrentFilterName();
    }

    bool selectFilter(const char* name) override {
        return abbot::filter::setCurrentFilterByName(name);
    }

    int getAvailableFilterCount() override {
        return abbot::filter::getAvailableFilterCount();
    }

    const char* getAvailableFilterName(int index) override {
        return abbot::filter::getAvailableFilterName(index);
    }

    void applyParamsFromPrefs() override {
        Preferences pref;
        if (!pref.begin("abbot", true)) {
            return;
        }

        auto f = getActiveFilter();
        const char *fname = getCurrentFilterName();
        if (!f || !fname) {
            pref.end();
            return;
        }

        if (strcmp(fname, "COMPLEMENTARY1D") == 0) {
            applyComplementary(pref, f);
        } else if (strcmp(fname, "MADGWICK") == 0) {
            applyMadgwick(pref, f);
        }

        pref.end();
    }

    void saveParamToPrefs(const char* key, float value) override {
        Preferences pref;
        if (pref.begin("abbot", false)) {
            pref.putFloat(key, value);
            pref.end();
        }
    }

    void requestWarmup(float seconds) override {
        requestTuningWarmupSeconds(seconds);
    }

private:
    void applyComplementary(Preferences& pref, abbot::IMUFilter* f) {
        bool applied = false;
        if (pref.isKey("c1d_kacc")) {
            float val = pref.getFloat("c1d_kacc", 0.02f);
            f->setParam("KACC", val);
            applied = true;
        }
        if (pref.isKey("c1d_kbias")) {
            float val = pref.getFloat("c1d_kbias", 0.01f);
            f->setParam("KBIAS", val);
            applied = true;
        }
        if (applied) {
            LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "FILTER: restored complementary params from NVS");
        }
    }

    void applyMadgwick(Preferences& pref, abbot::IMUFilter* f) {
        if (pref.isKey("mad_beta")) {
            float val = pref.getFloat("mad_beta", 0.021909f);
            f->setParam("BETA", val);
            LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "FILTER: restored Madgwick beta from NVS");
        }
    }
};

} // namespace abbot

#endif // FILTER_SERVICE_H
