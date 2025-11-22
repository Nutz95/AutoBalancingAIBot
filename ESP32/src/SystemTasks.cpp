// SystemTasks.cpp
#include "SystemTasks.h"
#include "BMI088Driver.h"
#include "imu_calibration.h"
#include "serial_commands.h"
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <Arduino.h>

namespace abbot {

static QueueHandle_t imuQueue = nullptr;
static BMI088Driver *g_driver = nullptr;
// Optional calibration queue (single-slot). When non-null the producer will also
// write each new sample into this queue (xQueueOverwrite) so a calibration routine
// can block-read exact samples without talking to the driver directly.
static QueueHandle_t calibQueue = nullptr;

static void imuProducerTask(void *pvParameters) {
  BMI088Driver *driver = reinterpret_cast<BMI088Driver*>(pvParameters);
  IMUSample sample;
  for (;;) {
    // Attempt to read; BMI088Driver::read will enforce sampling interval
    if (driver->read(sample)) {
      // Single-slot overwrite keeps only the latest sample
      if (imuQueue) {
        xQueueOverwrite(imuQueue, &sample);
      }
      // If a calibration queue is attached, push sample there as well (non-blocking)
      if (calibQueue) {
        xQueueOverwrite(calibQueue, &sample);
      }
    }
    // Short delay to yield; real timing enforced by driver->read
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

static void imuConsumerTask(void *pvParameters) {
  (void)pvParameters;
  IMUSample sample;
  uint32_t last_print_ms = 0;
  for (;;) {
    if (imuQueue && xQueueReceive(imuQueue, &sample, portMAX_DELAY) == pdTRUE) {
      #if defined(ENABLE_DEBUG_LOGS)
      // Suppress debug logs while calibration runs
      if (abbot::imu_cal::isCalibrating()) continue;
      // Throttle logging to once every 1000 ms to avoid flooding the serial
      uint32_t now = millis();
      if ((uint32_t)(now - last_print_ms) >= 1000u) {
        last_print_ms = now;
        Serial.print("IMU ts_ms="); Serial.print(sample.ts_ms);
        Serial.print(" ax="); Serial.print(sample.ax, 6);
        Serial.print(" ay="); Serial.print(sample.ay, 6);
        Serial.print(" az="); Serial.print(sample.az, 6);
        Serial.print(" gx="); Serial.print(sample.gx, 6);
        Serial.print(" gy="); Serial.print(sample.gy, 6);
        Serial.print(" gz="); Serial.println(sample.gz, 6);
      }
      #endif
    }
  }
}

bool startIMUTasks(BMI088Driver *driver) {
  if (!driver) return false;
  g_driver = driver;
  if (!imuQueue) {
    imuQueue = xQueueCreate(1, sizeof(IMUSample));
    if (!imuQueue) return false;
  }

  // Create producer task (higher priority)
  BaseType_t r1 = xTaskCreate(imuProducerTask, "IMUProducer", 4096, driver, configMAX_PRIORITIES - 2, nullptr);
  // Create consumer task (lower priority)
  BaseType_t r2 = xTaskCreate(imuConsumerTask, "IMUConsumer", 4096, nullptr, configMAX_PRIORITIES - 3, nullptr);

  // Create serial command task for calibration UI (low priority)
  BaseType_t r3 = xTaskCreate(abbot::serialcmds::serialTaskEntry, "IMUSerial", 4096, driver, configMAX_PRIORITIES - 4, nullptr);

  return (r1 == pdPASS) && (r2 == pdPASS) && (r3 == pdPASS);
}

void attachCalibrationQueue(QueueHandle_t q) {
  calibQueue = q;
}

void detachCalibrationQueue() {
  calibQueue = nullptr;
}

} // namespace abbot
