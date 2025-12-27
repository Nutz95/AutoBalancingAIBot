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
    /**
     * @brief Construct a new Log Command Handler object.
     */
    LogCommandHandler();
    virtual ~LogCommandHandler() = default;

    /**
     * @brief Get the command prefix ("LOG").
     * 
     * @return const char* The prefix string.
     */
    const char* getPrefix() const override { return "LOG"; }

    /**
     * @brief Handle log-specific commands.
     * 
     * @param line The original command line.
     * @param up The command line in uppercase.
     * @return true if the command was handled, false otherwise.
     */
    bool handleCommand(const String& line, const String& up) override;

    /**
     * @brief Build and return the log configuration menu.
     * 
     * @return SerialMenu* Pointer to the built menu (owned by this handler).
     */
    SerialMenu* buildMenu() override;

private:
    std::unique_ptr<SerialMenu> m_menu;
    bool handleLog(const String& line, const String& up);
    
    // Menu handlers
    void logMenuOnEnter();
};

} // namespace serialcmds
} // namespace abbot

#endif // LOG_COMMAND_HANDLER_H
