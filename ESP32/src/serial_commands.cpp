// serial_commands.cpp
#include "serial_commands.h"
#include "imu_calibration.h"
#include "motor_driver.h"
#include "BMI088Driver.h"
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace abbot {
namespace serialcmds {

static void processSerialOnce(class abbot::BMI088Driver *driver) {
  if (!driver) return;
  if (!Serial || Serial.available() == 0) return;
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;
  // First try imu_cal commands
  // imu_cal functions expect upper-case tokens; we forward the raw line but
  // the imu_cal module uppercases internally when parsing.
  // Try to handle CALIB commands first
  String up = line;
  up.toUpperCase();
  if (up.startsWith("CALIB")) {
    // reuse imu_cal parsing by calling its start/reset/dump functions
    // Copy-paste of previous parsing logic kept minimal here for clarity
    char buf[128];
    up.toCharArray(buf, sizeof(buf));
    char *tk = strtok(buf, " \t\r\n");
    if (!tk) return;
    char *t2 = strtok(NULL, " \t\r\n");
    if (!t2) return;
    if (strcmp(t2, "START") == 0) {
      char *what = strtok(NULL, " \t\r\n");
      if (!what) return;
      int sampleCount = 2000;
      char *nstr = strtok(NULL, " \t\r\n");
      if (nstr) {
        int v = atoi(nstr);
        if (v > 0) sampleCount = v;
      }
      if (strcmp(what, "GYRO") == 0) {
        if (!abbot::imu_cal::isCalibrating()) abbot::imu_cal::startGyroCalibration(*driver, sampleCount);
        return;
      } else if (strcmp(what, "ACCEL") == 0) {
        if (!abbot::imu_cal::isCalibrating()) abbot::imu_cal::startAccelCalibration(*driver, sampleCount);
        return;
      }
    } else if (strcmp(t2, "DUMP") == 0) {
      abbot::imu_cal::dumpCalibration();
      return;
    } else if (strcmp(t2, "RESET") == 0) {
      abbot::imu_cal::resetCalibration();
      return;
    }
  }

  // Forward to motor driver processor
  if (abbot::motor::processSerialCommand(line)) return;

  Serial.print("Unknown command: "); Serial.println(line);
}

void serialTaskEntry(void *pvParameters) {
  abbot::BMI088Driver *driver = reinterpret_cast<abbot::BMI088Driver*>(pvParameters);
  for (;;) {
    processSerialOnce(driver);
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

} // namespace serialcmds
} // namespace abbot
