#ifndef CALIBRATION_COMMAND_HANDLER_H
#define CALIBRATION_COMMAND_HANDLER_H

#include "ICommandHandler.h"
#include "../serial_menu.h"
#include "../BMI088Driver.h"

namespace abbot {
namespace serialcmds {

/**
 * @brief Handler for IMU and sensor calibration.
 * 
 * Manages commands starting with "CALIB" and provides a menu for
 * accelerometer, gyroscope, and magnetometer calibration.
 */
class CalibrationCommandHandler : public ICommandHandler {
public:
    CalibrationCommandHandler(abbot::BMI088Driver* driver);
    virtual ~CalibrationCommandHandler() = default;

    const char* getPrefix() const override { return "CALIB"; }
    bool handleCommand(const String& line, const String& up) override;
    SerialMenu* buildMenu() override;

private:
    SerialMenu* m_menu;
    bool handleCalib(const String& line, const String& up);
    
    // Menu handlers
    static void calibStartGyro(abbot::BMI088Driver *driver, const String &p);
    static void calibStartAccel(abbot::BMI088Driver *driver, const String &p);
    static void calibShow(abbot::BMI088Driver *driver);
    static void calibReset(abbot::BMI088Driver *driver);
    static void calibSave(abbot::BMI088Driver *driver);

    abbot::BMI088Driver* m_driver;
};

} // namespace serialcmds
} // namespace abbot

#endif // CALIBRATION_COMMAND_HANDLER_H
