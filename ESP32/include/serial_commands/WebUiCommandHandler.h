#pragma once

#include "serial_commands/ICommandHandler.h"

namespace abbot {
namespace serialcmds {

class WebUiCommandHandler : public ICommandHandler {
public:
    WebUiCommandHandler() = default;
    ~WebUiCommandHandler() override = default;

    const char* getPrefix() const override { return "WEBUI"; }
    bool handleCommand(const String& line, const String& lineUpper) override;
    SerialMenu* buildMenu() override { return nullptr; }

private:
    bool handleGetCommand(const String& lineUpper);
    static void emitControllerJson();
    static void emitFilterJson();
    static void emitMotorsJson();
    static void emitSystemJson();
    static void emitLogsJson();
};

} // namespace serialcmds
} // namespace abbot
