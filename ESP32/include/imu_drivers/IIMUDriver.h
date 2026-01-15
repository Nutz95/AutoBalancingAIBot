// IIMUDriver.h
#pragma once

#include "imu_types.h"
#include <stdint.h>

namespace abbot {

/**
 * @brief Interface for IMU drivers.
 * 
 * This interface allows the system to interact with different IMU hardware
 * (e.g., BMI088, BNO055) using a common API.
 */
class IIMUDriver {
public:
  virtual ~IIMUDriver() {
  }

  /**
   * @brief Initialize the IMU hardware.
   * @return true if initialization was successful, false otherwise.
   */
  virtual bool begin() = 0;

  /**
   * @brief Read the latest sample from the sensor.
   * @param out Reference to an IMUSample structure to be populated.
   * @return true if a new sample was read, false otherwise (e.g., not time yet).
   */
  virtual bool read(IMUSample &out) = 0;

  /**
   * @brief Immediate raw read from the sensor.
   * @param out Reference to an IMUSample structure to be populated.
   * @return true if the read was successful.
   */
  virtual bool readRaw(IMUSample &out) = 0;

  /**
   * @brief Get the configured sampling frequency in Hz.
   * @return The sampling frequency.
   */
  virtual uint16_t getSamplingHz() const = 0;

  /**
   * @brief Get the name of the driver.
   * @return A string representing the driver name.
   */
  virtual const char* getDriverName() const = 0;

  /**
   * @brief Check if the sensor is calibrated (useful for BNO055 internal calibration).
   * @return true if calibrated, false otherwise.
   */
  virtual bool isCalibrated() const {
    return true;
  }

  /**
   * @brief Get the pitch rate sign for balancing logic (1 or -1).
   */
  virtual int8_t getPitchRateSign() const = 0;
};

} // namespace abbot
