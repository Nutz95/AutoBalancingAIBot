#include <Arduino.h>
// Include BMI088 driver and IMU tasks
#include "BMI088Config.h"
#include "BMI088Driver.h"
#include "IMUTasks.h"

static abbot::BMI088Config bmi_cfg; 
static abbot::BMI088Driver bmi_driver(bmi_cfg);

void setup() {
        Serial.begin(921600);
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
        Serial.println("Hello ESP32-S3!");
        delay(1000);
}
