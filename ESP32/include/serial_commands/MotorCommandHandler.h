#pragma once

#include "serial_commands/ICommandHandler.h"
#include "../IMotorService.h"
#include <memory>

namespace abbot {
namespace serialcmds {

/**
 * @brief Handler for motor control and diagnostics.
 * 
 * Manages commands starting with "MOTOR" and provides a menu for
 * manual motor control, encoder reading, and telemetry monitoring.
 * 
 * This handler uses the IMotorService to interact with the motor hardware
 * and telemetry system, following the Dependency Inversion Principle.
 */
class MotorCommandHandler : public ICommandHandler {
public:
    /**
     * @brief Construct a new Motor Command Handler object.
     * 
     * @param motorService Pointer to the motor service implementation.
     */
    MotorCommandHandler(IMotorService* motorService);
    virtual ~MotorCommandHandler() = default;

    /**
     * @brief Get the command prefix ("MOTOR").
     * 
     * @return const char* The prefix string.
     */
    const char* getPrefix() const override { return "MOTOR"; }

    /**
     * @brief Handle motor-specific commands.
     * 
     * @param line The original command line.
     * @param lineUpper The command line in uppercase.
     * @return true if the command was handled, false otherwise.
     */
    bool handleCommand(const String& line, const String& lineUpper) override;

    /**
     * @brief Build and return the motor control menu.
     * 
     * @return SerialMenu* Pointer to the built menu (owned by this handler).
     */
    SerialMenu* buildMenu() override;

private:
    IMotorService* m_motorService;
    std::unique_ptr<SerialMenu> m_menu;

    void motorSetHandler(const String& p);
    bool handleMotorGetEncoder(const String& line, const String& up);
    bool handleMotorTelemetry(const String& line, const String& up);
};

} // namespace serialcmds
} // namespace abbot
