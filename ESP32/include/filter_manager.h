// filter_manager.h
#pragma once

#include "imu_filter.h"
#include "../config/FusionConfig.h"
#include <memory>

namespace abbot {
namespace filter {

// Initialize the filter manager and create a default active filter.
void init(const fusion::FusionConfig &cfg);

// Return the active filter instance (may be nullptr until init).
// Returned value is a `std::shared_ptr<IMUFilter>` so callers hold a reference
// while using the filter; this makes runtime swaps race-free.
std::shared_ptr<IMUFilter> getActiveFilter();

// Query available filters
int getAvailableFilterCount();
const char* getAvailableFilterName(int idx);

// Get/set current filter (name strings are simple ascii upper-case)
const char* getCurrentFilterName();
bool setCurrentFilterByName(const char* name);

} // namespace filter
} // namespace abbot
