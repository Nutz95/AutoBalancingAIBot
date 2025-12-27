#ifndef AUTOTUNE_COMMAND_HANDLER_H
#define AUTOTUNE_COMMAND_HANDLER_H

#include "ICommandHandler.h"
#include "../serial_menu.h"
#include "../IFusionService.h"

namespace abbot {
namespace serialcmds {

/**
 * @brief Handler for PID autotuning commands and menu.
 * 
 * Manages commands starting with "AUTOTUNE" and provides a menu for
 * starting, stopping, and configuring the PID autotuning process.
 */
class AutotuneCommandHandler : public ICommandHandler {
public:
    /**
     * @brief Construct a new Autotune Command Handler object.
     * 
     * @param fusionService Pointer to the fusion service implementation.
     */
    explicit AutotuneCommandHandler(IFusionService* fusionService);
    virtual ~AutotuneCommandHandler() = default;

    /**
     * @brief Get the command prefix ("AUTOTUNE").
     * 
     * @return const char* The prefix string.
     */
    const char* getPrefix() const override { return "AUTOTUNE"; }

    /**
     * @brief Handle autotune-specific commands.
     * 
     * @param line The original command line.
     * @param up The command line in uppercase.
     * @return true if the command was handled, false otherwise.
     */
    bool handleCommand(const String& line, const String& up) override;

    /**
     * @brief Build and return the autotune configuration menu.
     * 
     * @return SerialMenu* Pointer to the built menu (owned by this handler).
     */
    SerialMenu* buildMenu() override;

private:
    IFusionService* m_fusionService;
    std::unique_ptr<SerialMenu> m_menu;

    // Menu handlers
    void autotuneRelayHandler(const String& p);
    void autotuneDeadbandHandler(const String& p);
    void autotuneMaxAngleHandler(const String& p);
};

} // namespace serialcmds
} // namespace abbot

#endif // AUTOTUNE_COMMAND_HANDLER_H
