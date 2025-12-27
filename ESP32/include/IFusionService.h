#ifndef IFUSION_SERVICE_H
#define IFUSION_SERVICE_H

namespace abbot {

/**
 * @brief Interface for IMU fusion services.
 * 
 * Decouples command handlers from global fusion functions.
 */
class IFusionService {
public:
    virtual ~IFusionService() = default;

    virtual void printDiagnostics() = 0;
    virtual bool isReady() = 0;
    virtual unsigned long getWarmupRemaining() = 0;
    virtual void requestWarmup(float seconds) = 0;
    virtual float getPitch() = 0;
};

} // namespace abbot

#endif // IFUSION_SERVICE_H
