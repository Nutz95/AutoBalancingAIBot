#ifndef FILTER_COMMAND_HANDLER_H
#define FILTER_COMMAND_HANDLER_H

#include "ICommandHandler.h"
#include "../serial_menu.h"

namespace abbot {
namespace serialcmds {

/**
 * @brief Handler for IMU filter selection and parameter tuning.
 * 
 * Manages commands starting with "FILTER" and provides a menu for
 * switching between filters (Complementary, Kalman, Madgwick) and
 * adjusting their parameters.
 */
class FilterCommandHandler : public ICommandHandler {
public:
    FilterCommandHandler();
    virtual ~FilterCommandHandler() = default;

    const char* getPrefix() const override { return "FILTER"; }
    bool handleCommand(const String& line, const String& up) override;
    SerialMenu* buildMenu() override;

private:
    SerialMenu* m_menu;
    bool handleFilter(const String& line, const String& up);
    
    // Menu handlers
    static void filterSelectHandler(const String &p);
    static void filterParamSetHandler(const String &p);
};

} // namespace serialcmds
} // namespace abbot

#endif // FILTER_COMMAND_HANDLER_H
