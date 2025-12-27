#pragma once

#include "IMotorService.h"
#include "motor_drivers/driver_manager.h"
#include "serial_commands/motor_telemetry_manager.h"

namespace abbot {

class MotorService : public IMotorService {
public:
    abbot::motor::IMotorDriver* getActiveDriver() override {
        return abbot::motor::getActiveMotorDriver();
    }

    void startTelemetry(bool both, int intervalMs, bool leftSelected) override {
        abbot::serialcmds::g_telemetryManager.stop();
        abbot::serialcmds::g_telemetryManager.start(both, intervalMs, leftSelected);
    }

    void stopTelemetry() override {
        abbot::serialcmds::g_telemetryManager.stop();
    }

    bool isTelemetryRunning() const override {
        return abbot::serialcmds::g_telemetryManager.isRunning();
    }
};

} // namespace abbot
