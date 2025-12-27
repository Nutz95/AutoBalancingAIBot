#ifndef FILTER_COMMAND_HANDLER_H
#define FILTER_COMMAND_HANDLER_H

#include "ICommandHandler.h"
#include "../serial_menu.h"
#include "../IFilterService.h"

namespace abbot {
namespace serialcmds {

/**
 * @brief Handler for IMU filter selection and parameter tuning.
 * 
 * Manages commands starting with "FILTER" and provides a menu for
 * switching between filters (Complementary, Kalman, Madgwick) and
 * adjusting their parameters.
 * 
 * This handler uses the IFilterService to interact with the filter manager
 * and NVS storage, following the Dependency Inversion Principle.
 */
class FilterCommandHandler : public ICommandHandler {
public:
    /**
     * @brief Construct a new Filter Command Handler object.
     * 
     * @param filterService Pointer to the filter service implementation.
     */
    explicit FilterCommandHandler(IFilterService* filterService);
    virtual ~FilterCommandHandler() = default;

    /**
     * @brief Get the command prefix ("FILTER").
     * 
     * @return const char* The prefix string.
     */
    const char* getPrefix() const override { return "FILTER"; }

    /**
     * @brief Handle filter-specific commands.
     * 
     * @param line The original command line.
     * @param up The command line in uppercase.
     * @return true if the command was handled, false otherwise.
     */
    bool handleCommand(const String& line, const String& up) override;

    /**
     * @brief Build and return the filter configuration menu.
     * 
     * @return SerialMenu* Pointer to the built menu (owned by this handler).
     */
    SerialMenu* buildMenu() override;

private:
    IFilterService* m_filterService;
    std::unique_ptr<SerialMenu> m_menu;
    bool handleFilter(const String& line, const String& up);
    
    // Menu handlers
    void filterSelectHandler(const String &p);
    void filterParamSetHandler(const String &p);
};

} // namespace serialcmds
} // namespace abbot

#endif // FILTER_COMMAND_HANDLER_H
