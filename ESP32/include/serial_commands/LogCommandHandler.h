#ifndef LOG_COMMAND_HANDLER_H
#define LOG_COMMAND_HANDLER_H

#include "ICommandHandler.h"
#include "../serial_menu.h"

namespace abbot {
namespace serialcmds {

/**
 * @brief Handler for system logging configuration.
 * 
 * Manages commands starting with "LOG" and provides a menu for
 * enabling/disabling log categories and setting log levels.
 */
class LogCommandHandler : public ICommandHandler {
public:
    LogCommandHandler();
    virtual ~LogCommandHandler() = default;

    const char* getPrefix() const override { return "LOG"; }
    bool handleCommand(const String& line, const String& up) override;
    SerialMenu* buildMenu() override;

private:
    SerialMenu* m_menu;
    bool handleLog(const String& line, const String& up);
    
    // Menu handlers
    void logMenuOnEnter();
};

} // namespace serialcmds
} // namespace abbot

#endif // LOG_COMMAND_HANDLER_H
