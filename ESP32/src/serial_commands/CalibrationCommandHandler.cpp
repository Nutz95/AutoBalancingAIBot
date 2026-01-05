#include "serial_commands/CalibrationCommandHandler.h"
#include "imu_calibration.h"
#include "logging.h"
#include <Arduino.h>

namespace abbot {
namespace serialcmds {

CalibrationCommandHandler::CalibrationCommandHandler(abbot::IIMUDriver* driver)
    : m_driver(driver) {
    m_menu.reset(new SerialMenu("Calibration Commands"));
    m_menu->addEntry(1, "CALIB START GYRO [N]",
                [driver](const String &p) { calibStartGyro(driver, p); });
    m_menu->addEntry(2, "CALIB START ACCEL [N]",
                [driver](const String &p) { calibStartAccel(driver, p); });
    m_menu->addEntry(3, "CALIB DUMP",
                [](const String &) { abbot::imu_cal::dumpCalibration(); });
    m_menu->addEntry(4, "CALIB RESET",
                [](const String &) { abbot::imu_cal::resetCalibration(); });
}

bool CalibrationCommandHandler::handleCommand(const String& line, const String& up) {
    if (!up.startsWith("CALIB")) {
        return false;
    }

    String s = up;
    s.trim();
    if (s == "CALIB") {
        LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "CALIB usage: CALIB START GYRO [N] | CALIB START ACCEL [N] | CALIB DUMP | CALIB RESET");
        return true;
    }

    if (s.startsWith("CALIB START GYRO")) {
        calibStartGyro(m_driver, line.substring(16));
        return true;
    } else if (s.startsWith("CALIB START ACCEL")) {
        calibStartAccel(m_driver, line.substring(17));
        return true;
    } else if (s == "CALIB DUMP") {
        abbot::imu_cal::dumpCalibration();
        return true;
    } else if (s == "CALIB RESET") {
        abbot::imu_cal::resetCalibration();
        return true;
    }

    return false;
}

SerialMenu* CalibrationCommandHandler::buildMenu() {
    return m_menu.get();
}

void CalibrationCommandHandler::calibStartGyro(abbot::IIMUDriver *driver, const String &p) {
  int n = 4000;
  String s = p;
  s.trim();
  if (s.length()) {
    n = s.toInt();
  }
  if (!abbot::imu_cal::isCalibrating()) {
    abbot::log::pushChannelMask(
        static_cast<uint32_t>(abbot::log::CHANNEL_DEFAULT) |
        static_cast<uint32_t>(abbot::log::CHANNEL_IMU));
    abbot::imu_cal::startGyroCalibration(*driver, n);
    abbot::log::popChannelMask();
  }
}

void CalibrationCommandHandler::calibStartAccel(abbot::IIMUDriver *driver, const String &p) {
  int n = 4000;
  String s = p;
  s.trim();
  if (s.length()) {
    n = s.toInt();
  }
  if (!abbot::imu_cal::isCalibrating()) {
    abbot::log::pushChannelMask(
        static_cast<uint32_t>(abbot::log::CHANNEL_DEFAULT) |
        static_cast<uint32_t>(abbot::log::CHANNEL_IMU));
    abbot::imu_cal::startAccelCalibration(*driver, n);
    abbot::log::popChannelMask();
  }
}

} // namespace serialcmds
} // namespace abbot
