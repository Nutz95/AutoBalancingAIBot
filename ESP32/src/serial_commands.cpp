// serial_commands.cpp
#include "serial_commands.h"
#include "imu_calibration.h"
#include "motor_driver.h"
#include "BMI088Driver.h"
#include "logging.h"
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "SystemTasks.h"

namespace abbot {
namespace serialcmds {

void processSerialOnce(class abbot::BMI088Driver *driver) {
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

  // HELP command: print list of available serial commands
  if (up == "HELP" || up == "?" || up.startsWith("HELP ")) {
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "Available commands:");
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "  CALIB START GYRO [N]   - start gyro calibration (N samples, default 2000)");
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "  CALIB START ACCEL [N]  - start accel calibration (N samples, default 2000)");
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "  CALIB DUMP             - show current calibration values (gyro_bias, accel_offset)");
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "  CALIB RESET            - reset stored calibration (clear NVS)");
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "  MOTOR ENABLE           - enable motors (initializes servo bus on-demand and enables torque)");
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "  MOTOR DISABLE          - disable motors (torque off)");
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "  MOTOR STATUS           - print motor enabled state and IDs");
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "  MOTOR DUMP             - dump motor config (pins, IDs)");
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "  TUNING START           - start CSV tuning stream (timestamp,pitch_deg,pitch_rad,pitch_rate_deg,pitch_rate_rad,left_cmd,right_cmd)");
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "  TUNING STOP            - stop CSV tuning stream");
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "  MOTOR READ <LEFT|RIGHT|ID>     - read encoder/position from servo");
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "  MOTOR SET <LEFT|RIGHT|ID> <v>  - set motor command normalized in [-1.0..1.0]");
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "      Normalized values are mapped to servo units using SC_SERVO_MAX_SPEED (default 7000). Use small values first (e.g. 0.05)");
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "  MOTOR SET <LEFT|RIGHT|ID> RAW <value> - send raw signed servo speed units (bypasses normalization)");
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "      RAW is useful for precise tuning (e.g. RAW 2000). Use short pulses and be careful.");
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "  MOTOR PARAMS <LEFT|RIGHT|ID> - read and print servo EEPROM/SRAM parameters and present status");
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "  HELP or ?              - print this help text");
    return;
  }

  // TUNING commands: start/stop CSV stream
  if (up.startsWith("TUNING")) {
    char buf2[32];
    up.toCharArray(buf2, sizeof(buf2));
    char *tk = strtok(buf2, " \t\r\n");
    char *arg = strtok(NULL, " \t\r\n");
    if (arg && strcmp(arg, "START") == 0) {
      abbot::startTuningStream();
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "TUNING: started");
      return;
    } else if (arg && strcmp(arg, "STOP") == 0) {
      abbot::stopTuningStream();
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "TUNING: stopped");
      return;
    } else {
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "TUNING usage: TUNING START | TUNING STOP");
      return;
    }
  }

  // LOG commands: enable/disable/list channels
  if (up.startsWith("LOG")) {
    char buf3[64];
    up.toCharArray(buf3, sizeof(buf3));
    char *tk = strtok(buf3, " \t\r\n");
    char *cmd = strtok(NULL, " \t\r\n");
    if (!cmd) {
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "LOG usage: LOG ENABLE|DISABLE <CHANNEL> | LOG LIST");
      return;
    }
    if (strcmp(cmd, "LIST") == 0) {
      char out[128];
      abbot::log::listEnabledChannels(out, sizeof(out));
      LOG_PRINT(abbot::log::CHANNEL_DEFAULT, "LOG enabled: "); LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, out);
      return;
    }
    char *argch = strtok(NULL, " \t\r\n");
    if (!argch) {
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "LOG usage: LOG ENABLE|DISABLE <CHANNEL>");
      return;
    }
    auto ch = abbot::log::channelFromString(argch);
    if (ch == static_cast<abbot::log::Channel>(0)) {
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "Unknown channel. Known: TUNING,BLE,IMU,MOTOR,DEFAULT");
      return;
    }
    if (strcmp(cmd, "ENABLE") == 0) {
      abbot::log::enableChannel(ch);
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "LOG: enabled");
      return;
    } else if (strcmp(cmd, "DISABLE") == 0) {
      abbot::log::disableChannel(ch);
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "LOG: disabled");
      return;
    }
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "LOG usage: LOG ENABLE|DISABLE <CHANNEL> | LOG LIST");
    return;
  }

  // Forward to motor driver processor
  if (abbot::motor::processSerialCommand(line)) return;

  LOG_PRINT(abbot::log::CHANNEL_DEFAULT, "Unknown command: "); LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, line);
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
