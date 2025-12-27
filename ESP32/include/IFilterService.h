#ifndef IFILTER_SERVICE_H
#define IFILTER_SERVICE_H

#include <Arduino.h>
#include "imu_filter.h"

namespace abbot {

/**
 * @brief Interface for IMU filter management.
 * 
 * Decouples command handlers from the global filter manager state.
 */
class IFilterService {
public:
    virtual ~IFilterService() = default;

    /**
     * @brief Get the currently active filter.
     */
    virtual abbot::IMUFilter* getActiveFilter() = 0;

    /**
     * @brief Get the name of the currently active filter.
     */
    virtual const char* getCurrentFilterName() = 0;

    /**
     * @brief Select a filter by name.
     */
    virtual bool selectFilter(const char* name) = 0;

    /**
     * @brief Get the number of available filters.
     */
    virtual int getAvailableFilterCount() = 0;

    /**
     * @brief Get the name of an available filter by index.
     */
    virtual const char* getAvailableFilterName(int index) = 0;

    /**
     * @brief Apply filter parameters from NVS.
     */
    virtual void applyParamsFromPrefs() = 0;

    /**
     * @brief Save a filter parameter to NVS.
     */
    virtual void saveParamToPrefs(const char* key, float value) = 0;

    /**
     * @brief Request a warmup period for the IMU.
     */
    virtual void requestWarmup(float seconds) = 0;
};

} // namespace abbot

#endif // IFILTER_SERVICE_H
