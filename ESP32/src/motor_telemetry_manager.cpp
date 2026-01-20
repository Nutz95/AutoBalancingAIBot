#include "serial_commands/motor_telemetry_manager.h"
#include <Arduino.h>

namespace abbot {
namespace serialcmds {

void MotorTelemetryManager::taskEntry(void *pv) {
  MotorTelemetryManager *telemetryManager = static_cast<MotorTelemetryManager *>(pv);
  while (telemetryManager->running) {
    uint64_t timestamp_us = (uint64_t)esp_timer_get_time();  // Use microseconds for consistency
    auto activeDriver = abbot::motor::getActiveMotorDriver();
    if (activeDriver) {
      const char* drvName = activeDriver->getDriverName();
      uint32_t busLatUs = activeDriver->getLastBusLatencyUs();
      uint32_t accel = activeDriver->getSpeedCommandAccel();
      if (telemetryManager->both) {
        int leftId = abbot::motor::getActiveMotorId(abbot::motor::IMotorDriver::MotorSide::LEFT, -1);
        int rightId = abbot::motor::getActiveMotorId(abbot::motor::IMotorDriver::MotorSide::RIGHT, -1);
        // Read values using driver-manager helpers (they internally protect access).
        int32_t leftEncoder = abbot::motor::readEncoderBySide(abbot::motor::IMotorDriver::MotorSide::LEFT);
        int32_t rightEncoder = abbot::motor::readEncoderBySide(abbot::motor::IMotorDriver::MotorSide::RIGHT);
        float leftSpeed = abbot::motor::readSpeedBySide(abbot::motor::IMotorDriver::MotorSide::LEFT);
        float rightSpeed = abbot::motor::readSpeedBySide(abbot::motor::IMotorDriver::MotorSide::RIGHT);
        uint64_t leftCmdTs = 0;
        uint64_t rightCmdTs = 0;
        // Safely read last command timestamps using the previously obtained
        // `activeDriver` pointer. Avoid exceptions in MCU build; drivers may
        // return 0 when timestamping is not supported.
        if (activeDriver) {
          leftCmdTs = activeDriver->getLastCommandTimeUs(abbot::motor::IMotorDriver::MotorSide::LEFT);
          rightCmdTs = activeDriver->getLastCommandTimeUs(abbot::motor::IMotorDriver::MotorSide::RIGHT);
        }
        int64_t leftAgeMs = (leftCmdTs > 0 && timestamp_us >= leftCmdTs) ? (int64_t)((timestamp_us - leftCmdTs) / 1000ULL) : -1;
        int64_t rightAgeMs = (rightCmdTs > 0 && timestamp_us >= rightCmdTs) ? (int64_t)((timestamp_us - rightCmdTs) / 1000ULL) : -1;
        LOG_PRINTF(abbot::log::CHANNEL_MOTOR,
             "MOTOR: telemetry drv=%s ts_us=%llu interval=%dms bus_lat_us=%lu accel=%lu L(id=%d) enc=%ld sp=%.2f cmd_age_ms=%lld R(id=%d) enc=%ld sp=%.2f cmd_age_ms=%lld\n",
             drvName, (unsigned long long)timestamp_us, telemetryManager->intervalMs, (unsigned long)busLatUs, (unsigned long)accel,
                   leftId, (long)leftEncoder, (double)leftSpeed, (long long)leftAgeMs,
                   rightId, (long)rightEncoder, (double)rightSpeed, (long long)rightAgeMs);
      } else {
        int selectedSideId = telemetryManager->singleLeft ?
            abbot::motor::getActiveMotorId(abbot::motor::IMotorDriver::MotorSide::LEFT, -1) :
            abbot::motor::getActiveMotorId(abbot::motor::IMotorDriver::MotorSide::RIGHT, -1);
        abbot::motor::IMotorDriver::MotorSide side = telemetryManager->singleLeft ?
            abbot::motor::IMotorDriver::MotorSide::LEFT : abbot::motor::IMotorDriver::MotorSide::RIGHT;
        // Read encoder and speed for the selected side
        int32_t encoderValue = abbot::motor::readEncoderBySide(side);
        float speedValue = abbot::motor::readSpeedBySide(side);
        uint64_t cmdTs = 0;
        if (activeDriver) {
          cmdTs = activeDriver->getLastCommandTimeUs(side);
        }
        int64_t ageMs = (cmdTs > 0 && timestamp_us >= cmdTs) ? (int64_t)((timestamp_us - cmdTs) / 1000ULL) : -1;
        LOG_PRINTF(abbot::log::CHANNEL_MOTOR,
             "MOTOR: telemetry drv=%s ts_us=%llu interval=%dms bus_lat_us=%lu accel=%lu id=%d enc=%ld sp=%.2f cmd_age_ms=%lld\n",
             drvName, (unsigned long long)timestamp_us, telemetryManager->intervalMs, (unsigned long)busLatUs, (unsigned long)accel,
                   selectedSideId, (long)encoderValue, (double)speedValue, (long long)ageMs);
      }
    } else {
      LOG_PRINTLN(abbot::log::CHANNEL_MOTOR, "MOTOR: telemetry (no driver)");
    }
    vTaskDelay(pdMS_TO_TICKS(telemetryManager->intervalMs));
  }
  // clear task handle before exit
  telemetryManager->taskHandle = nullptr;
  vTaskDelete(nullptr);
}

MotorTelemetryManager::MotorTelemetryManager() {
  // Constructor: no runtime work required — members use in-class initializers.
}

MotorTelemetryManager::~MotorTelemetryManager() {
  // Ensure the telemetry task is stopped before destruction.
  stop();
}

void MotorTelemetryManager::start(bool bothSides, int ms, bool leftSide) {
  stop();
  if (ms <= 0) {
    return;
  }
  both = bothSides;
  singleLeft = leftSide;
  intervalMs = ms;
  running = true;
  BaseType_t rc = xTaskCreate(&taskEntry, "motor_telemetry", 3072, this, tskIDLE_PRIORITY + 1, &taskHandle);
  if (rc != pdPASS) {
    LOG_PRINTLN(abbot::log::CHANNEL_MOTOR, "Failed to start telemetry task");
    running = false;
    taskHandle = nullptr;
  }
  // Reset driver speed estimators so telemetry starts from a fresh state.
  auto activeDrv = abbot::motor::getActiveMotorDriver();
  if (activeDrv) {
    activeDrv->resetSpeedEstimator();
  }
}

void MotorTelemetryManager::stop() {
  if (!running && taskHandle == nullptr) {
    return;
  }
  running = false;
  // Wait for the task to exit (task clears taskHandle)
  for (int i = 0; i < 50 && taskHandle; ++i) {
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  if (taskHandle) {
    vTaskDelete(taskHandle);
    taskHandle = nullptr;
  }
}

bool MotorTelemetryManager::isRunning() const {
  return running;
}

MotorTelemetryManager g_telemetryManager;

} // namespace serialcmds
} // namespace abbot
