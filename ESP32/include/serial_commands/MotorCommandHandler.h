#pragma once

#include "serial_commands/ICommandHandler.h"

namespace abbot {
namespace serialcmds {

/**
 * @brief Handler for motor control and diagnostics.
 * 
 * Manages commands starting with "MOTOR" and provides a menu for
 * manual motor control, encoder reading, and telemetry monitoring.
 */
class MotorCommandHandler : public ICommandHandler {
public:
    MotorCommandHandler() = default;
    virtual ~MotorCommandHandler() = default;

    const char* getPrefix() const override { return "MOTOR"; }
    bool handleCommand(const String& line, const String& lineUpper) override;
    SerialMenu* buildMenu() override;

private:
    void motorSetHandler(const String& p);
    bool handleMotorGetEncoder(const String& line, const String& up);
    bool handleMotorTelemetry(const String& line, const String& up);
};

} // namespace serialcmds
} // namespace abbot
