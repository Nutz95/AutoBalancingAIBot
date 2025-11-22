// IMUTasks.cpp
#include "IMUTasks.h"
#include "BMI088Driver.h"
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <Arduino.h>

namespace abbot {

static QueueHandle_t imuQueue = nullptr;
static BMI088Driver *g_driver = nullptr;

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
      // Throttle logging to once every 100 ms to avoid flooding the serial
      uint32_t now = millis();
      if ((uint32_t)(now - last_print_ms) >= 100u) {
        last_print_ms = now;
        Serial.print("IMU ts_ms="); Serial.print(sample.ts_ms);
        Serial.print(" ax="); Serial.print(sample.ax, 6);
        Serial.print(" ay="); Serial.print(sample.ay, 6);
        Serial.print(" az="); Serial.print(sample.az, 6);
        Serial.print(" gx="); Serial.print(sample.gx, 6);
        Serial.print(" gy="); Serial.print(sample.gy, 6);
        Serial.print(" gz="); Serial.println(sample.gz, 6);
      }
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

  return (r1 == pdPASS) && (r2 == pdPASS);
}

} // namespace abbot
