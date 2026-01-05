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
    /**
     * @brief Construct a new Balancer Command Handler object.
     * 
     * @param fusionService Pointer to the fusion service implementation.
     */
    BalancerCommandHandler(IFusionService* fusionService);
    virtual ~BalancerCommandHandler() = default;

    /**
     * @brief Get the command prefix ("BALANCE").
     * 
     * @return const char* The prefix string.
     */
    const char* getPrefix() const override { return "BALANCE"; }

    /**
     * @brief Handle balancer-specific commands.
     * 
     * @param line The original command line.
     * @param up The command line in uppercase.
     * @return true if the command was handled, false otherwise.
     */
    bool handleCommand(const String& line, const String& up) override;

    /**
     * @brief Build and return the balancer configuration menu.
     * 
     * @return SerialMenu* Pointer to the built menu (owned by this handler).
     */
    SerialMenu* buildMenu() override;

private:
    std::unique_ptr<SerialMenu> m_menu;
    IFusionService* m_fusionService;
    bool handleBalance(const String& line, const String& up);
    
    // Menu handlers
    void balancerStartHandler(const String& p);
    static void balancerSetGainsHandler(const String& p);
    static void balancerDeadbandSetHandler(const String& p);
    static void balancerMinCmdSetHandler(const String& p);
    static void balancerMotorGainsSetHandler(const String& p);
    static void balancerAdaptiveSetHandler(const String& p);
    static void balancerCalibrateStartHandler(const String& p);
};

} // namespace serialcmds
} // namespace abbot

#endif // BALANCER_COMMAND_HANDLER_H
