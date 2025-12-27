#pragma once

#include <Arduino.h>
#include "motor_drivers/IMotorDriver.h"

namespace abbot {

/**
 * @brief Interface for motor control and telemetry services.
 * 
 * Decouples command handlers from global motor driver and telemetry state.
 */
class IMotorService {
public:
    virtual ~IMotorService() = default;

    virtual abbot::motor::IMotorDriver* getActiveDriver() = 0;
    
    virtual void startTelemetry(bool both, int intervalMs, bool leftSelected) = 0;
    virtual void stopTelemetry() = 0;
    virtual bool isTelemetryRunning() const = 0;
};

} // namespace abbot
