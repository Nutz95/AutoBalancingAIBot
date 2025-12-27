#ifndef AUTOTUNE_COMMAND_HANDLER_H
#define AUTOTUNE_COMMAND_HANDLER_H

#include "ICommandHandler.h"
#include "../serial_menu.h"

namespace abbot {
namespace serialcmds {

/**
 * @brief Handler for PID autotuning commands and menu.
 */
class AutotuneCommandHandler : public ICommandHandler {
public:
    AutotuneCommandHandler();
    virtual ~AutotuneCommandHandler() = default;

    const char* getPrefix() const override { return "AUTOTUNE"; }
    bool handleCommand(const String& line, const String& up) override;
    SerialMenu* buildMenu() override;

private:
    SerialMenu* m_menu;

    // Menu handlers
    static void autotuneRelayHandler(const String& p);
    static void autotuneDeadbandHandler(const String& p);
    static void autotuneMaxAngleHandler(const String& p);
};

} // namespace serialcmds
} // namespace abbot

#endif // AUTOTUNE_COMMAND_HANDLER_H
