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
    TuningCommandHandler();
    virtual ~TuningCommandHandler() = default;

    const char* getPrefix() const override { return "TUNING"; }
    bool handleCommand(const String& line, const String& up) override;
    SerialMenu* buildMenu() override;

    SerialMenu* getMenu();

private:
    bool handleTuning(const String& line, const String& up);
    
    // Menu handlers
    static void tuningCustomHandler(const String &p);
    static void tuningStreamStartHandler(const String &p);
    static void tuningStreamStopHandler(const String &p);

    SerialMenu* m_menu;
};

} // namespace serialcmds
} // namespace abbot

#endif // TUNING_COMMAND_HANDLER_H
