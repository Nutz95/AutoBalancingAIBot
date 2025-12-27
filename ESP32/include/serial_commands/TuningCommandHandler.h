#ifndef TUNING_COMMAND_HANDLER_H
#define TUNING_COMMAND_HANDLER_H

#include "ICommandHandler.h"
#include "../serial_menu.h"

namespace abbot {
namespace serialcmds {

/**
 * @brief Handler for real-time tuning and data streaming.
 * 
 * Manages commands starting with "TUNING" and provides a menu for
 * starting/stopping data streams used for PID tuning and system analysis.
 */
class TuningCommandHandler : public ICommandHandler {
public:
    /**
     * @brief Construct a new Tuning Command Handler object.
     */
    TuningCommandHandler();
    virtual ~TuningCommandHandler() = default;

    /**
     * @brief Get the command prefix ("TUNING").
     * 
     * @return const char* The prefix string.
     */
    const char* getPrefix() const override { return "TUNING"; }

    /**
     * @brief Handle tuning-specific commands.
     * 
     * @param line The original command line.
     * @param up The command line in uppercase.
     * @return true if the command was handled, false otherwise.
     */
    bool handleCommand(const String& line, const String& up) override;

    /**
     * @brief Build and return the tuning configuration menu.
     * 
     * @return SerialMenu* Pointer to the built menu (owned by this handler).
     */
    SerialMenu* buildMenu() override;

    /**
     * @brief Get the menu object.
     * 
     * @return SerialMenu* Pointer to the menu.
     */
    SerialMenu* getMenu();

private:
    std::unique_ptr<SerialMenu> m_menu;
    bool handleTuning(const String& line, const String& up);

    // Menu handlers
    void tuningStreamStartHandler(const String &p);
    void tuningStreamStopHandler(const String &p);
    void tuningCustomHandler(const String &p);
};

} // namespace serialcmds
} // namespace abbot

#endif // TUNING_COMMAND_HANDLER_H
