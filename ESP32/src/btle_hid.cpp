// Clean NimBLE HID gamepad reader
#include "btle_hid.h"
#include "../config/controller_config.h"       // Button mapping configuration
#include "../config/motor_configs/servo_motor_config.h" // LEFT_MOTOR_ID / RIGHT_MOTOR_ID
#include "SystemTasks.h"
#include "balancer_controller.h" // for balance toggle
#include "btle_callbacks.h"
#include "logging.h"
#include "motor_drivers/driver_manager.h"
#include <NimBLEDevice.h>
#include <cstdio>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <vector>

// Helper: wait for the NimBLE client to report connected state (with timeout).
// Returns the number of milliseconds actually waited.
static uint32_t waitForClientConnected(NimBLEClient *client,
                                       uint32_t timeout_ms,
                                       uint32_t step_ms = 50) {
  uint32_t waited = 0;
  while (waited < timeout_ms) {
    if (client && client->isConnected() && g_connected) {
      break;
    }
    vTaskDelay(pdMS_TO_TICKS(step_ms));
    waited += step_ms;
  }
  return waited;
}

// Helper: after requesting secure connection, wait until connection stabilises
// or until a timeout/ disconnect occurs. Returns milliseconds waited.
static uint32_t waitForStableAfterSecure(NimBLEClient *client,
                                         uint32_t timeout_ms,
                                         uint32_t step_ms = 50) {
  uint32_t waited = 0;
  while (waited < timeout_ms) {
    if (!client || !client->isConnected() || !g_connected) {
      // disconnected -> stop waiting
      break;
    }
    vTaskDelay(pdMS_TO_TICKS(step_ms));
    waited += step_ms;
  }
  return waited;
}

NimBLEClient *g_client = nullptr;
NimBLEAdvertisedDevice *g_targetDevice = nullptr;
volatile bool g_connected = false;

// Track button states for edge detection (one byte per tracked button)
static uint8_t g_lastBalanceButtonState = 0;
static uint8_t g_lastTrimButtonState = 0;

// Handle controller button presses
static void handleControllerButtons(const uint8_t *pData, size_t length) {
  using namespace controller_config;

  if (length < MIN_HID_REPORT_LENGTH) {
    return;
  }

  // --- Balance Toggle Button ---
  if (length > BALANCE_TOGGLE_BYTE) {
    uint8_t currentButton = pData[BALANCE_TOGGLE_BYTE] & BALANCE_TOGGLE_MASK;
    uint8_t wasPressed = g_lastBalanceButtonState;

    if (currentButton != wasPressed) {
      char buf[96];
      snprintf(buf, sizeof(buf),
               "BLE BTN balance byte=%u mask=0x%02X state=%s",
               static_cast<unsigned>(BALANCE_TOGGLE_BYTE),
               static_cast<unsigned>(BALANCE_TOGGLE_MASK),
               currentButton ? "pressed" : "released");
      LOG_PRINTLN(abbot::log::CHANNEL_BLE, buf);
    }

    // Balance toggle: falling edge (button released)
    if (wasPressed && !currentButton) {
      if (abbot::balancer::controller::isActive()) {
        abbot::balancer::controller::stop();
        if (auto driver = abbot::motor::getActiveMotorDriver()) {
          driver->disableMotors();
        }
        LOG_PRINTLN(abbot::log::CHANNEL_BLE,
                    ">>> BALANCE STOP (controller button)");
      } else {
        if (auto driver = abbot::motor::getActiveMotorDriver()) {
          if (!driver->areMotorsEnabled()) {
            driver->enableMotors();
          }
        }
        abbot::balancer::controller::start(abbot::getFusedPitch());
        LOG_PRINTLN(abbot::log::CHANNEL_BLE,
                    ">>> BALANCE START (controller button)");
      }
    }
    g_lastBalanceButtonState = currentButton;
  }

  // --- Trim Calibrate Button ---
  if (length > TRIM_CALIBRATE_BYTE) {
    uint8_t currentButton = pData[TRIM_CALIBRATE_BYTE] & TRIM_CALIBRATE_MASK;
    uint8_t wasPressed = g_lastTrimButtonState;

    if (currentButton != wasPressed) {
      char buf[96];
      snprintf(buf, sizeof(buf),
               "BLE BTN trim byte=%u mask=0x%02X state=%s",
               static_cast<unsigned>(TRIM_CALIBRATE_BYTE),
               static_cast<unsigned>(TRIM_CALIBRATE_MASK),
               currentButton ? "pressed" : "released");
      LOG_PRINTLN(abbot::log::CHANNEL_BLE, buf);
    }

    // Trim calibrate: falling edge (button released)
    if (wasPressed && !currentButton) {
      abbot::balancer::controller::calibrateTrim();
      LOG_PRINTLN(abbot::log::CHANNEL_BLE,
                  ">>> TRIM CALIBRATED (controller button)");
    }
    g_lastTrimButtonState = currentButton;
  }
}

void notifyCallback(NimBLERemoteCharacteristic *pChar, uint8_t *pData,
                    size_t length, bool isNotify) {
  // Log raw HID report for button mapping discovery
  static uint8_t lastData[32] = {0};
  bool changed = false;
  for (size_t i = 0; i < length && i < 32; i++) {
    if (pData[i] != lastData[i]) {
      changed = true;
      break;
    }
  }

  if (changed) {
    char buf[128];
    int pos = snprintf(buf, sizeof(buf), "HID[%d]: ", (int)length);
    for (size_t i = 0; i < length && i < 16 && pos < 120; i++) {
      pos += snprintf(buf + pos, sizeof(buf) - pos, "%02X ", pData[i]);
    }
    LOG_PRINTLN(abbot::log::CHANNEL_BLE, buf);
    memcpy(lastData, pData, length < 32 ? length : 32);
  }

  // Handle button actions (balance toggle, etc.)
  handleControllerButtons(pData, length);

  if (length < 4) {
    return;
  }

  // Xbox controller HID format (16 bytes):
  // Bytes 0-1: Left stick X (little-endian, unsigned 16-bit)
  // Bytes 2-3: Left stick Y (little-endian, unsigned 16-bit)
  // Measured center: X=0x82F9 (33529), Y=0x72A7 (29351)
  // Range: 0x0000 to 0xFFFF

  uint16_t rawX = (uint16_t)(pData[0] | (pData[1] << 8));
  uint16_t rawY = (uint16_t)(pData[2] | (pData[3] << 8));

  // Calibrated center values from testing
  const float centerX = 33529.0f;
  const float centerY = 29351.0f;
  const float rangeX = 32768.0f; // Half of 16-bit range
  const float rangeY = 32768.0f;

  // Convert to -1.0 to +1.0 range with calibrated center
  float x = ((float)rawX - centerX) / rangeX;
  float y = ((float)rawY - centerY) / rangeY;

  // Clamp to valid range
  if (x > 1.0f)
    x = 1.0f;
  if (x < -1.0f)
    x = -1.0f;
  if (y > 1.0f)
    y = 1.0f;
  if (y < -1.0f)
    y = -1.0f;

  // Apply deadzone
  if (fabs(x) < 0.12f)
    x = 0.0f;
  if (fabs(y) < 0.12f)
    y = 0.0f;

  // Invert Y (up should be positive for forward)
  y = -y;

  // Tank drive: forward = y, turn = x
  float forward = y;
  // Disable turning for now (user requested). Keep placeholder scale if
  // re-enabled.
  float turn = 0.0f; // x * 0.6f; // Reduce turn sensitivity

  float leftMotor = forward + turn;
  float rightMotor = forward - turn;

  // Clamp to [-1, 1]
  if (leftMotor > 1.0f)
    leftMotor = 1.0f;
  if (leftMotor < -1.0f)
    leftMotor = -1.0f;
  if (rightMotor > 1.0f)
    rightMotor = 1.0f;
  if (rightMotor < -1.0f)
    rightMotor = -1.0f;

  // Only print and send if stick moved significantly
  static float lastLeft = 0, lastRight = 0;
  if (fabs(leftMotor - lastLeft) > 0.05f ||
      fabs(rightMotor - lastRight) > 0.05f) {
    LOG_PRINTF(abbot::log::CHANNEL_BLE,
               "\n>>> Stick X:%.2f Y:%.2f -> L:%.2f R:%.2f\n", x, y, leftMotor,
               rightMotor);

    // If balancer controller is active, route stick forward/back through
    // the controller so it can convert v -> pitch setpoint safely.
    if (abbot::balancer::controller::isActive()) {
      // Send normalized forward and turn (turn currently forced zero)
      abbot::balancer::controller::setDriveSetpoints(forward, 0.0f);
    } else {
      // When the balancer is inactive, ignore stick movement to prevent
      // unintended wheel motion before BALANCE START. Keep motors disabled
      // until the operator explicitly starts balancing.
      LOG_PRINTLN(abbot::log::CHANNEL_MOTOR,
                  "motor_driver: stick ignored (balancer inactive)");
    }
    // Keep logging for debug
    LOG_PRINTF(abbot::log::CHANNEL_MOTOR, "MOTOR DBG LEFT %.3f\n", leftMotor);
    LOG_PRINTF(abbot::log::CHANNEL_MOTOR, "MOTOR DBG RIGHT %.3f\n", rightMotor);

    lastLeft = leftMotor;
    lastRight = rightMotor;
  }
}

// Background BLE task: performs scanning and connection loops. Runs as FreeRTOS
// task.
static void btleTask(void *pvParameters) {
  (void)pvParameters;
  NimBLEScan *pScan = NimBLEDevice::getScan();
  pScan->setAdvertisedDeviceCallbacks(new AdvCallbacks());
  pScan->setActiveScan(true);
  pScan->setInterval(100);
  pScan->setWindow(99);

  for (;;) {
    if (!g_connected) {
      g_targetDevice = nullptr;
      LOG_PRINTLN(abbot::log::CHANNEL_BLE, "Scanning for HID devices (10s)...");
      pScan->start(10, false);

      if (g_targetDevice) {
        LOG_PRINTLN(abbot::log::CHANNEL_BLE, ">>> Found target, connecting...");

        g_client = NimBLEDevice::createClient();
        g_client->setClientCallbacks(new ClientCallbacks());
        g_client->setConnectTimeout(5);

        if (g_client->connect(g_targetDevice)) {
          LOG_PRINTLN(abbot::log::CHANNEL_BLE,
                      ">>> connect() returned true, discovering services...");

          // Wait up to 500ms for the client to report connected state.
          {
            const uint32_t waited = waitForClientConnected(g_client, 500);
            LOG_PRINTF(abbot::log::CHANNEL_BLE,
                       ">>> waited %ums for connection state\n", waited);
          }

          LOG_PRINTLN(abbot::log::CHANNEL_BLE,
                      ">>> Initiating secure connection...");
          g_client->secureConnection();

          // After requesting secure connection, wait up to 1000ms for the
          // link to stabilise. Bail out early if the client disconnects.
          {
            const uint32_t waited = waitForStableAfterSecure(g_client, 1000);
            LOG_PRINTF(abbot::log::CHANNEL_BLE,
                       ">>> waited %ums after secureConnection\n", waited);
          }

          NimBLERemoteService *pService =
              g_client->getService(NimBLEUUID((uint16_t)0x1812));
          if (pService) {
            LOG_PRINTLN(abbot::log::CHANNEL_BLE, ">>> Found HID service");
            std::vector<NimBLERemoteCharacteristic *> *pCharacteristics =
                pService->getCharacteristics(true);
            LOG_PRINTF(abbot::log::CHANNEL_BLE, ">>> Characteristics: %u\n",
                       (unsigned)pCharacteristics->size());
            for (auto pChar : *pCharacteristics) {
              LOG_PRINTF(abbot::log::CHANNEL_BLE, "  %s - R:%s W:%s N:%s\n",
                         pChar->getUUID().toString().c_str(),
                         pChar->canRead() ? "Y" : "N",
                         pChar->canWrite() ? "Y" : "N",
                         pChar->canNotify() ? "Y" : "N");
              if (pChar->getUUID() == NimBLEUUID((uint16_t)0x2A4D) &&
                  pChar->canNotify()) {
                LOG_PRINTLN(abbot::log::CHANNEL_BLE,
                            "    -> Subscribing to Input Report...");
                if (pChar->subscribe(true, notifyCallback)) {
                  LOG_PRINTLN(abbot::log::CHANNEL_BLE,
                              "    -> Subscribe SUCCESS!");
                } else {
                  LOG_PRINTLN(abbot::log::CHANNEL_BLE,
                              "    -> Subscribe FAILED!");
                }
              }
            }
          } else {
            LOG_PRINTLN(abbot::log::CHANNEL_BLE, ">>> HID service NOT found!");
          }
        } else {
          LOG_PRINTLN(abbot::log::CHANNEL_BLE, ">>> connect() returned FALSE!");
        }
      } else {
        LOG_PRINTLN(abbot::log::CHANNEL_BLE,
                    ">>> No HID device found, retrying...");
      }

      vTaskDelay(pdMS_TO_TICKS(2000));
    } else {
      // Connected, just wait
      vTaskDelay(pdMS_TO_TICKS(1000));
      LOG_PRINT(abbot::log::CHANNEL_BLE, ".");
    }
  }
}

void abbot::btle_hid::begin() {
  LOG_PRINTLN(abbot::log::CHANNEL_BLE, "\n=== NimBLE HID Scanner ===");

  NimBLEDevice::init("");
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);

  // Configure security for pairing
  NimBLEDevice::setSecurityAuth(true, true, true);           // bond, MITM, SC
  NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT); // No PIN needed

  LOG_PRINTLN(abbot::log::CHANNEL_BLE,
              ">>> Security configured: bonding enabled, no PIN");

  // Create a background FreeRTOS task to run the BLE scan/connect loop so
  // `begin()` returns quickly and does not block `setup()`.
  BaseType_t r = xTaskCreate(btleTask, "BTLETask", 8192, nullptr,
                             configMAX_PRIORITIES - 5, nullptr);
  if (r != pdPASS) {
    LOG_PRINTLN(abbot::log::CHANNEL_BLE, "Failed to create BTLE task");
  }
}
