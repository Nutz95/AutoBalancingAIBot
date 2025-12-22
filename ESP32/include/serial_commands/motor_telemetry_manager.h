#pragma once
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "logging.h"
#include "motor_drivers/driver_manager.h"

namespace abbot {
namespace serialcmds {

class MotorTelemetryManager {
public:
  /**
   * MotorTelemetryManager
   * Manages a FreeRTOS task that periodically emits motor telemetry lines
   * on the logging channel. Thread-safe start/stop control is provided.
   */
  MotorTelemetryManager();
  ~MotorTelemetryManager();

  /**
   * Start telemetry emission.
   * @param bothSides  true to report both motors, false to report a single side
   * @param ms         interval in milliseconds (must be > 0)
   * @param leftSide   when not bothSides, select left (true) or right (false)
   * Note: calling start() stops any existing telemetry task first.
   */
  void start(bool bothSides, int ms, bool leftSide = true);

  /**
   * Stop telemetry emission. This may block briefly while the task exits.
   */
  void stop();

  /**
   * Returns true when telemetry task is running.
   */
  bool isRunning() const;

private:
  // internal state (kept private but accessed by the implementation)
  TaskHandle_t taskHandle = nullptr;
  volatile bool running = false;
  int intervalMs = 0;
  bool both = false;
  bool singleLeft = true;

  // task entry (static so it can be passed to FreeRTOS)
  static void taskEntry(void *pv);
};

// Global instance used by serial command handlers.
extern MotorTelemetryManager g_telemetryManager;

} // namespace serialcmds
} // namespace abbot
