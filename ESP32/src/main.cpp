#include <Arduino.h>
// Include BMI088 driver and IMU tasks
#include "../config/BMI088Config.h"
#include "BMI088Driver.h"
#include "SystemTasks.h"
#include "imu_calibration.h"

static abbot::BMI088Config bmi_cfg; 
static abbot::BMI088Driver bmi_driver(bmi_cfg);

void setup() {
        Serial.begin(921600);
        // Print calibration values if present
        {
            abbot::imu_cal::Calibration cal;
            if (abbot::imu_cal::loadCalibration(cal)) {
                Serial.print("Calibration loaded: gyro_bias=");
                Serial.print(cal.gyro_bias[0], 6); Serial.print(','); Serial.print(cal.gyro_bias[1], 6); Serial.print(','); Serial.println(cal.gyro_bias[2], 6);
                Serial.print("Calibration loaded: accel_offset=");
                Serial.print(cal.accel_offset[0], 6); Serial.print(','); Serial.print(cal.accel_offset[1], 6); Serial.print(','); Serial.println(cal.accel_offset[2], 6);
                // Install into the imu_cal module so `CALIB DUMP` reports these values
                abbot::imu_cal::installCalibration(cal);
            } else {
                Serial.println("No IMU calibration found");
            }
        }
        // initialize BMI088 driver
        bool ok = bmi_driver.begin();
        if (!ok) {
            Serial.println("BMI088 init failed");
        } else {
            Serial.println("BMI088 initialized");
        }

        // start IMU producer/consumer tasks for demo
        if (abbot::startIMUTasks(&bmi_driver)) {
            Serial.println("IMU tasks started");
        } else {
            Serial.println("Failed to start IMU tasks");
        }
}

void loop() {
        // Main loop can perform other duties. Keep alive message for demo.
        delay(1000);
}
