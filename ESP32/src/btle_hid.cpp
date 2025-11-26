// Clean NimBLE HID gamepad reader
#include "btle_hid.h"
#include "SystemTasks.h"
#include <NimBLEDevice.h>
#include "motor_driver.h" // auto-enable motors when controller active
#include "../config/motor_config.h" // LEFT_MOTOR_ID / RIGHT_MOTOR_ID
#include "logging.h"
#include "btle_callbacks.h"

NimBLEClient* g_client = nullptr;
NimBLEAdvertisedDevice* g_targetDevice = nullptr;
volatile bool g_connected = false;
void notifyCallback(NimBLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify) {
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
  if(x > 1.0f) x = 1.0f;
  if(x < -1.0f) x = -1.0f;
  if(y > 1.0f) y = 1.0f;
  if(y < -1.0f) y = -1.0f;
  
  // Apply deadzone
  if(fabs(x) < 0.12f) x = 0.0f;
  if(fabs(y) < 0.12f) y = 0.0f;
  
  // Invert Y (up should be positive for forward)
  y = -y;
  
  // Tank drive: forward = y, turn = x
  float forward = y;
  float turn = x * 0.6f; // Reduce turn sensitivity
  
  float leftMotor = forward + turn;
  float rightMotor = forward - turn;
  
  // Clamp to [-1, 1]
  if(leftMotor > 1.0f) leftMotor = 1.0f;
  if(leftMotor < -1.0f) leftMotor = -1.0f;
  if(rightMotor > 1.0f) rightMotor = 1.0f;
  if(rightMotor < -1.0f) rightMotor = -1.0f;
  
  // Auto-enable motors on first meaningful stick movement
  if (!abbot::motor::areMotorsEnabled() && (fabs(x) > 0.01f || fabs(y) > 0.01f)) {
    LOG_PRINTLN(abbot::log::CHANNEL_MOTOR, "motor_driver: auto-enable due to controller input");
    abbot::motor::enableMotors();
  }

  // Only print and send if stick moved significantly
  static float lastLeft = 0, lastRight = 0;
  if (fabs(leftMotor - lastLeft) > 0.05f || fabs(rightMotor - lastRight) > 0.05f) {
    LOG_PRINTF(abbot::log::CHANNEL_BLE, "\n>>> Stick X:%.2f Y:%.2f -> L:%.2f R:%.2f\n", x, y, leftMotor, rightMotor);

    // Direct motor driver invocation (avoid loopback via Serial)
    if (abbot::motor::areMotorsEnabled()) {
      abbot::motor::setMotorCommand(LEFT_MOTOR_ID, leftMotor);
      abbot::motor::setMotorCommand(RIGHT_MOTOR_ID, rightMotor);
    } else {
      LOG_PRINTLN(abbot::log::CHANNEL_MOTOR, "motor_driver: movement ignored (motors disabled)");
    }
    // Keep logging for debug
    LOG_PRINTF(abbot::log::CHANNEL_MOTOR, "MOTOR DBG LEFT %.3f\n", leftMotor);
    LOG_PRINTF(abbot::log::CHANNEL_MOTOR, "MOTOR DBG RIGHT %.3f\n", rightMotor);

    lastLeft = leftMotor;
    lastRight = rightMotor;
  }
}

void abbot::btle_hid::begin() {
  LOG_PRINTLN(abbot::log::CHANNEL_BLE, "\n=== NimBLE HID Scanner ===");

  NimBLEDevice::init("");
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);

  // Configure security for pairing
  NimBLEDevice::setSecurityAuth(true, true, true); // bond, MITM, SC
  NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT); // No PIN needed

  LOG_PRINTLN(abbot::log::CHANNEL_BLE, ">>> Security configured: bonding enabled, no PIN");

  NimBLEScan* pScan = NimBLEDevice::getScan();
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
          LOG_PRINTLN(abbot::log::CHANNEL_BLE, ">>> connect() returned true, discovering services...");

          // Wait a bit for connection to stabilize
          delay(500);

          // Initiate pairing/encryption
          LOG_PRINTLN(abbot::log::CHANNEL_BLE, ">>> Initiating secure connection...");
          g_client->secureConnection();
          delay(1000); // Wait for pairing to complete

          NimBLERemoteService* pService = g_client->getService(NimBLEUUID((uint16_t)0x1812));
          if (pService) {
            LOG_PRINTLN(abbot::log::CHANNEL_BLE, ">>> Found HID service");

            std::vector<NimBLERemoteCharacteristic*>* pCharacteristics = pService->getCharacteristics(true);
            LOG_PRINTF(abbot::log::CHANNEL_BLE, ">>> Characteristics: %u\n", (unsigned)pCharacteristics->size());

            for (auto pChar : *pCharacteristics) {
              LOG_PRINTF(abbot::log::CHANNEL_BLE, "  %s - R:%s W:%s N:%s\n",
                         pChar->getUUID().toString().c_str(),
                         pChar->canRead() ? "Y" : "N",
                         pChar->canWrite() ? "Y" : "N",
                         pChar->canNotify() ? "Y" : "N");

              // Subscribe to Input Report (0x2A4D) with notify
              if (pChar->getUUID() == NimBLEUUID((uint16_t)0x2A4D) && pChar->canNotify()) {
                LOG_PRINTLN(abbot::log::CHANNEL_BLE, "    -> Subscribing to Input Report...");
                if (pChar->subscribe(true, notifyCallback)) {
                  LOG_PRINTLN(abbot::log::CHANNEL_BLE, "    -> Subscribe SUCCESS!");
                } else {
                  LOG_PRINTLN(abbot::log::CHANNEL_BLE, "    -> Subscribe FAILED!");
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
        LOG_PRINTLN(abbot::log::CHANNEL_BLE, ">>> No HID device found, retrying...");
      }

      delay(2000);
      } else {
        // Connected, just wait
        delay(1000);
        LOG_PRINT(abbot::log::CHANNEL_BLE, ".");
      }
  }
}
