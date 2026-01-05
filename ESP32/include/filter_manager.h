// filter_manager.h
#pragma once

#include "../config/FusionConfig.h"
#include "imu_filter.h"
#include <memory>

namespace abbot {
namespace filter {

/**
 * @brief Initialize the filter manager and create a default active filter.
 * 
 * This should be called during system startup. It loads the previously used
 * filter from non-volatile storage if available.
 * 
 * @param cfg The fusion configuration to use for the filters.
 */
void init(const fusion::FusionConfig &cfg);

/**
 * @brief Return the active filter instance.
 * 
 * The returned value is a `std::shared_ptr<IMUFilter>` so callers hold a 
 * reference while using the filter; this makes runtime swaps race-free.
 * 
 * @return std::shared_ptr<IMUFilter> The currently active filter, or nullptr.
 */
std::shared_ptr<IMUFilter> getActiveFilter();

/**
 * @brief Get the number of available filters.
 * @return int The count of filters in s_entries.
 */
int getAvailableFilterCount();

/**
 * @brief Get the name of a filter by its index.
 * @param index The index of the filter.
 * @return const char* The name of the filter, or empty string if index is invalid.
 */
const char *getAvailableFilterName(int index);

/**
 * @brief Get the name of the currently active filter.
 * @return const char* The name of the active filter.
 */
const char *getCurrentFilterName();

/**
 * @brief Set the current filter by its name.
 * 
 * This will instantiate the new filter, initialize it with the current 
 * configuration, and persist the choice to NVS.
 * 
 * @param name The uppercase name of the filter (e.g., "MADGWICK").
 * @return true if the filter was successfully found and set, false otherwise.
 */
bool setCurrentFilterByName(const char *name);

} // namespace filter
} // namespace abbot
