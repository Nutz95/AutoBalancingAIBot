#include "serial_commands/motor_telemetry_manager.h"
#include "motor_drivers/critical_guard.h"
#include <Arduino.h>

namespace abbot {
namespace serialcmds {

void MotorTelemetryManager::taskEntry(void *pv) {
  MotorTelemetryManager *telemetryManager = static_cast<MotorTelemetryManager *>(pv);
  while (telemetryManager->running) {
    unsigned long timestamp = millis();
    auto activeDriver = abbot::motor::getActiveMotorDriver();
    if (activeDriver) {
      if (telemetryManager->both) {
        int leftId = abbot::motor::getActiveMotorId(abbot::motor::IMotorDriver::MotorSide::LEFT, -1);
        int rightId = abbot::motor::getActiveMotorId(abbot::motor::IMotorDriver::MotorSide::RIGHT, -1);
        // Read values using driver-manager helpers (they internally protect access).
        int32_t leftEncoder = abbot::motor::readEncoderBySide(abbot::motor::IMotorDriver::MotorSide::LEFT);
        int32_t rightEncoder = abbot::motor::readEncoderBySide(abbot::motor::IMotorDriver::MotorSide::RIGHT);
        float leftSpeed = abbot::motor::readSpeedBySide(abbot::motor::IMotorDriver::MotorSide::LEFT);
        float rightSpeed = abbot::motor::readSpeedBySide(abbot::motor::IMotorDriver::MotorSide::RIGHT);
        LOG_PRINTF(abbot::log::CHANNEL_MOTOR,
                   "MOTOR: telemetry ts=%lu interval=%dms L(id=%d) enc=%ld sp=%.2f R(id=%d) enc=%ld sp=%.2f\n",
                   (unsigned long)timestamp, telemetryManager->intervalMs, leftId, (long)leftEncoder,
                   (double)leftSpeed, rightId, (long)rightEncoder, (double)rightSpeed);
      } else {
        int selectedSideId = telemetryManager->singleLeft ?
            abbot::motor::getActiveMotorId(abbot::motor::IMotorDriver::MotorSide::LEFT, -1) :
            abbot::motor::getActiveMotorId(abbot::motor::IMotorDriver::MotorSide::RIGHT, -1);
        abbot::motor::IMotorDriver::MotorSide side = telemetryManager->singleLeft ?
            abbot::motor::IMotorDriver::MotorSide::LEFT : abbot::motor::IMotorDriver::MotorSide::RIGHT;
        // Read encoder and speed for the selected side
        int32_t encoderValue = abbot::motor::readEncoderBySide(side);
        float speedValue = abbot::motor::readSpeedBySide(side);
        LOG_PRINTF(abbot::log::CHANNEL_MOTOR,
                   "MOTOR: telemetry ts=%lu interval=%dms id=%d enc=%ld sp=%.2f\n",
                   (unsigned long)timestamp, telemetryManager->intervalMs, selectedSideId,
                   (long)encoderValue, (double)speedValue);
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

MotorTelemetryManager::MotorTelemetryManager() {}

MotorTelemetryManager::~MotorTelemetryManager() {
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
