#ifndef BALANCER_COMMAND_HANDLER_H
#define BALANCER_COMMAND_HANDLER_H

#include "ICommandHandler.h"
#include "../serial_menu.h"
#include "../IFusionService.h"

namespace abbot {
namespace serialcmds {

/**
 * @brief Handler for balancer PID control and trim.
 * 
 * Manages commands starting with "BALANCE" and provides a menu for
 * PID tuning, deadband calibration, and trim adjustment.
 */
class BalancerCommandHandler : public ICommandHandler {
public:
    BalancerCommandHandler(IFusionService* fusion);
    virtual ~BalancerCommandHandler() = default;

    const char* getPrefix() const override { return "BALANCE"; }
    bool handleCommand(const String& line, const String& up) override;
    SerialMenu* buildMenu() override;

private:
    SerialMenu* m_menu;
    IFusionService* m_fusion;
    bool handleBalance(const String& line, const String& up);
    
    // Menu handlers
    void balancerStartHandler(const String& p);
    static void balancerSetGainsHandler(const String& p);
    static void balancerDeadbandSetHandler(const String& p);
    static void balancerMinCmdSetHandler(const String& p);
    static void balancerMotorGainsSetHandler(const String& p);
};

} // namespace serialcmds
} // namespace abbot

#endif // BALANCER_COMMAND_HANDLER_H
