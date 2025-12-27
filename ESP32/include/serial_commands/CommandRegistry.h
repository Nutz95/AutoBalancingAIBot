#pragma once

#include "ICommandHandler.h"
#include <vector>
#include <memory>

namespace abbot {
namespace serialcmds {

/**
 * @brief Registry and dispatcher for serial commands.
 * 
 * This class manages a collection of command handlers and dispatches
 * incoming serial lines to the appropriate handler based on command prefixes.
 */
class CommandRegistry {
public:
    /**
     * @brief Registers a new command handler.
     * 
     * @param handler Unique pointer to the handler. The registry takes ownership.
     */
    void registerHandler(std::unique_ptr<ICommandHandler> handler) {
        if (handler) {
            _handlers.push_back(std::move(handler));
        }
    }

    /**
     * @brief Dispatches a command line to the registered handlers.
     * 
     * @param line The original command line.
     * @param lineUpper Optional pre-calculated uppercase version of the line.
     *                  If empty, it will be calculated internally.
     * @return true if a handler processed the command, false otherwise.
     */
    bool dispatch(const String& line, const String& lineUpper = "") {
        const String* upPtr = &lineUpper;
        String localUpper;

        if (lineUpper.length() == 0) {
            localUpper = line;
            localUpper.toUpperCase();
            localUpper.trim();
            upPtr = &localUpper;
        }

        if (upPtr->length() == 0) {
            return false;
        }

        for (auto& handler : _handlers) {
            if (upPtr->startsWith(handler->getPrefix())) {
                if (handler->handleCommand(line, *upPtr)) {
                    return true;
                }
            }
        }
        return false;
    }

    /**
     * @brief Returns all registered handlers.
     */
    const std::vector<std::unique_ptr<ICommandHandler>>& getHandlers() const {
        return _handlers;
    }

private:
    std::vector<std::unique_ptr<ICommandHandler>> _handlers;
};

} // namespace serialcmds
} // namespace abbot
