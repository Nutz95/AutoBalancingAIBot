#pragma once

#include "serial_commands/ICommandHandler.h"

namespace abbot {
namespace serialcmds {

/**
 * @brief Handler for WiFi configuration and status.
 * 
 * Manages commands starting with "WIFI" and provides a menu for
 * connecting to access points, checking status, and configuring the web console.
 */
class WifiCommandHandler : public ICommandHandler {
public:
    WifiCommandHandler() = default;
    virtual ~WifiCommandHandler() = default;

    const char* getPrefix() const override { return "WIFI"; }
    bool handleCommand(const String& line, const String& lineUpper) override;
    SerialMenu* buildMenu() override;
};

} // namespace serialcmds
} // namespace abbot
