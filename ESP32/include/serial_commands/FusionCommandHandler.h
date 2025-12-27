#ifndef FUSION_COMMAND_HANDLER_H
#define FUSION_COMMAND_HANDLER_H

#include "ICommandHandler.h"
#include "../IFusionService.h"

namespace abbot {
namespace serialcmds {

/**
 * @brief Handler for IMU fusion diagnostics and warmup.
 */
class FusionCommandHandler : public ICommandHandler {
public:
    FusionCommandHandler(IFusionService* service);
    virtual ~FusionCommandHandler() = default;

    bool handleCommand(const String& line, const String& up) override;
    const char* getPrefix() const override { return "FUSION"; }
    SerialMenu* buildMenu() override;

private:
    IFusionService* m_service;
};

} // namespace serialcmds
} // namespace abbot

#endif // FUSION_COMMAND_HANDLER_H
