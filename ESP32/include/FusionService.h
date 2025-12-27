#ifndef FUSION_SERVICE_IMPL_H
#define FUSION_SERVICE_IMPL_H

#include "IFusionService.h"
#include "imu_fusion.h"
#include "SystemTasks.h"

namespace abbot {

class FusionService : public IFusionService {
public:
    void printDiagnostics() override {
        abbot::printMadgwickDiagnostics();
    }

    bool isReady() override {
        return abbot::isFusionReady();
    }

    unsigned long getWarmupRemaining() override {
        return abbot::getFusionWarmupRemaining();
    }

    void requestWarmup(float seconds) override {
        requestTuningWarmupSeconds(seconds);
    }

    float getPitch() override {
        return abbot::getFusedPitch();
    }
};

} // namespace abbot

#endif // FUSION_SERVICE_IMPL_H
