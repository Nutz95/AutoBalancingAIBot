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
    /**
     * @brief Construct a new Calibration Command Handler object.
     * 
     * @param driver Pointer to the BMI088 driver instance.
     */
    CalibrationCommandHandler(abbot::BMI088Driver* driver);
    virtual ~CalibrationCommandHandler() = default;

    /**
     * @brief Get the command prefix ("CALIB").
     * 
     * @return const char* The prefix string.
     */
    const char* getPrefix() const override { return "CALIB"; }

    /**
     * @brief Handle calibration-specific commands.
     * 
     * @param line The original command line.
     * @param up The command line in uppercase.
     * @return true if the command was handled, false otherwise.
     */
    bool handleCommand(const String& line, const String& up) override;

    /**
     * @brief Build and return the calibration menu.
     * 
     * @return SerialMenu* Pointer to the built menu (owned by this handler).
     */
    SerialMenu* buildMenu() override;

private:
    std::unique_ptr<SerialMenu> m_menu;
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
