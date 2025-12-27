#ifndef FUSION_COMMAND_HANDLER_H
#define FUSION_COMMAND_HANDLER_H

#include "ICommandHandler.h"
#include "../IFusionService.h"

namespace abbot {
namespace serialcmds {

/**
 * @brief Handler for IMU fusion diagnostics and warmup.
 * 
 * Manages commands starting with "FUSION" and provides status
 * information and warmup control for the IMU fusion system.
 */
class FusionCommandHandler : public ICommandHandler {
public:
    /**
     * @brief Construct a new Fusion Command Handler object.
     * 
     * @param service Pointer to the fusion service implementation.
     */
    FusionCommandHandler(IFusionService* service);
    virtual ~FusionCommandHandler() = default;

    /**
     * @brief Handle fusion-specific commands.
     * 
     * @param line The original command line.
     * @param up The command line in uppercase.
     * @return true if the command was handled, false otherwise.
     */
    bool handleCommand(const String& line, const String& up) override;

    /**
     * @brief Get the command prefix ("FUSION").
     * 
     * @return const char* The prefix string.
     */
    const char* getPrefix() const override { return "FUSION"; }

    /**
     * @brief Build and return the fusion menu.
     * 
     * @return SerialMenu* Pointer to the built menu (owned by this handler).
     */
    SerialMenu* buildMenu() override;

private:
    IFusionService* m_service;
};

} // namespace serialcmds
} // namespace abbot

#endif // FUSION_COMMAND_HANDLER_H
