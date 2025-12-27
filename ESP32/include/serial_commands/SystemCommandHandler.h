#pragma once

#include "serial_commands/ICommandHandler.h"
#include <memory>

namespace abbot {
namespace serialcmds {

/**
 * @brief Handler for system-level commands.
 * 
 * Manages commands like REBOOT, HEAP, and TASKS.
 */
class SystemCommandHandler : public ICommandHandler {
public:
    /**
     * @brief Construct a new System Command Handler object.
     */
    SystemCommandHandler() = default;
    virtual ~SystemCommandHandler() = default;

    /**
     * @brief Get the command prefix ("SYS").
     * 
     * @return const char* The prefix string.
     */
    const char* getPrefix() const override { return "SYS"; }

    /**
     * @brief Handle system-specific commands.
     * 
     * @param line The original command line.
     * @param lineUpper The command line in uppercase.
     * @return true if the command was handled, false otherwise.
     */
    bool handleCommand(const String& line, const String& lineUpper) override;

    /**
     * @brief Build and return the system commands menu.
     * 
     * @return SerialMenu* Pointer to the built menu (owned by this handler).
     */
    SerialMenu* buildMenu() override;

private:
    std::unique_ptr<SerialMenu> m_menu;
};

} // namespace serialcmds
} // namespace abbot
