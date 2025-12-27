#pragma once

#include <Arduino.h>
#include "serial_menu.h"

namespace abbot {
namespace serialcmds {

/**
 * @brief Interface for domain-specific command handlers.
 * 
 * Each module (Motor, WiFi, etc.) should implement this interface to handle
 * its own textual commands and provide its interactive menu.
 */
class ICommandHandler {
public:
    virtual ~ICommandHandler() = default;

    /**
     * @brief Returns the command prefix this handler is interested in (e.g., "MOTOR").
     * 
     * The registry uses this to quickly filter commands.
     * @return const char* The uppercase prefix string.
     */
    virtual const char* getPrefix() const = 0;

    /**
     * @brief Handles a textual command line.
     * 
     * @param line The original command line (preserved case).
     * @param lineUpper The command line converted to uppercase for easier matching.
     * @return true if the command was handled by this module, false otherwise.
     */
    virtual bool handleCommand(const String& line, const String& lineUpper) = 0;

    /**
     * @brief Builds and returns the interactive menu for this module.
     * 
     * The handler retains ownership of the SerialMenu object. The caller
     * should not delete the returned pointer.
     * 
     * @return SerialMenu* The menu object, or nullptr if no menu is provided.
     */
    virtual SerialMenu* buildMenu() = 0;
};

} // namespace serialcmds
} // namespace abbot
