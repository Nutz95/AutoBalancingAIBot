#include <Arduino.h>
// Include BMI088 driver and IMU tasks
#include "../config/BMI088Config.h"
#include "BMI088Driver.h"
#include "SystemTasks.h"
#include "imu_calibration.h"
#include "motor_driver.h"
#include "../config/motor_config.h"

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
        // initialize motor driver early so the servo bus is ready for serial commands
        abbot::motor::initMotorDriver();
        // Log motor driver status and attempt to read encoder/status for both motors
        Serial.print("motor_driver: enabled="); Serial.println(abbot::motor::areMotorsEnabled() ? "YES" : "NO");
    #if MOTOR_DRIVER_REAL
        {
            int32_t left_pos = abbot::motor::readEncoder(LEFT_MOTOR_ID);
            int32_t right_pos = abbot::motor::readEncoder(RIGHT_MOTOR_ID);
            Serial.print("motor_driver: encoder LEFT_ID="); Serial.print(LEFT_MOTOR_ID); Serial.print(" pos="); Serial.println(left_pos);
            Serial.print("motor_driver: encoder RIGHT_ID="); Serial.print(RIGHT_MOTOR_ID); Serial.print(" pos="); Serial.println(right_pos);
        }
    #endif

        if (abbot::startIMUTasks(&bmi_driver)) {
            Serial.println("IMU tasks started");
        } else {
            Serial.println("Failed to start IMU tasks");
        }
        // Diagnostic heartbeat to verify serial output during boot
        Serial.println("BOOT DEBUG: setup complete, entering main loop");
}

void loop() {
            // Main loop can perform other duties. Heartbeat confirms Serial is alive.
            delay(1000);
}
