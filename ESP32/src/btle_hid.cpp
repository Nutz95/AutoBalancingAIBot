// Clean NimBLE HID gamepad reader
#include "btle_hid.h"
#include <NimBLEDevice.h>
#include "motor_driver.h" // auto-enable motors when controller active
#include "../config/motor_config.h" // LEFT_MOTOR_ID / RIGHT_MOTOR_ID

static NimBLEClient* g_client = nullptr;
static NimBLEAdvertisedDevice* g_targetDevice = nullptr;
static volatile bool g_connected = false;

static void notifyCallback(NimBLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify) {
  if(length < 4) return;
  
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
  if(!abbot::motor::areMotorsEnabled() && (fabs(x) > 0.01f || fabs(y) > 0.01f)) {
    Serial.println("motor_driver: auto-enable due to controller input");
    abbot::motor::enableMotors();
  }

  // Only print and send if stick moved significantly
  static float lastLeft = 0, lastRight = 0;
  if(fabs(leftMotor - lastLeft) > 0.05f || fabs(rightMotor - lastRight) > 0.05f) {
    Serial.print("\n>>> Stick X:"); Serial.print(x, 2);
    Serial.print(" Y:"); Serial.print(y, 2);
    Serial.print(" -> L:"); Serial.print(leftMotor, 2);
    Serial.print(" R:"); Serial.println(rightMotor, 2);
    
    // Direct motor driver invocation (avoid loopback via Serial)
    if (abbot::motor::areMotorsEnabled()) {
      abbot::motor::setMotorCommand(LEFT_MOTOR_ID, leftMotor);
      abbot::motor::setMotorCommand(RIGHT_MOTOR_ID, rightMotor);
    } else {
      Serial.println("motor_driver: movement ignored (motors disabled)");
    }
    // Keep logging for debug
    Serial.print("MOTOR DBG LEFT "); Serial.println(leftMotor,3);
    Serial.print("MOTOR DBG RIGHT "); Serial.println(rightMotor,3);
    
    lastLeft = leftMotor;
    lastRight = rightMotor;
  }
}

class ClientCallbacks : public NimBLEClientCallbacks {
  void onConnect(NimBLEClient* pClient) {
    g_connected = true;
    Serial.println(">>> BLE: Connected callback fired");
    pClient->updateConnParams(12, 24, 0, 60);
  }
  
  void onDisconnect(NimBLEClient* pClient) {
    g_connected = false;
    Serial.println(">>> BLE: Disconnected");
    // Auto-disable motors when controller link is lost
    if (abbot::motor::areMotorsEnabled()) {
      Serial.println("motor_driver: auto-disable due to controller disconnect");
      abbot::motor::disableMotors();
    }
  }
  
  void onAuthenticationComplete(ble_gap_conn_desc* desc) {
    Serial.print(">>> AUTH complete - encrypted: ");
    Serial.println(desc->sec_state.encrypted ? "YES" : "NO");
    if(desc->sec_state.encrypted) {
      Serial.println(">>> Link is now encrypted, data should flow!");
    }
  }
  
  bool onConfirmPIN(uint32_t pin) {
    Serial.print(">>> Confirm PIN: "); Serial.println(pin);
    return true;
  }
};

class AdvCallbacks : public NimBLEAdvertisedDeviceCallbacks {
  void onResult(NimBLEAdvertisedDevice* advertisedDevice) {
    Serial.print("Found: ");
    Serial.print(advertisedDevice->getName().c_str());
    Serial.print(" [");
    Serial.print(advertisedDevice->getAddress().toString().c_str());
    Serial.print("]");
    
    if(advertisedDevice->isAdvertisingService(NimBLEUUID((uint16_t)0x1812))) {
      Serial.println(" -> HID");
      g_targetDevice = advertisedDevice;
      NimBLEDevice::getScan()->stop();
    } else {
      Serial.println();
    }
  }
};

void abbot::btle_hid::begin() {
  Serial.println("\n=== NimBLE HID Scanner ===");
  
  NimBLEDevice::init("");
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);
  
  // Configure security for pairing
  NimBLEDevice::setSecurityAuth(true, true, true); // bond, MITM, SC
  NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT); // No PIN needed
  
  Serial.println(">>> Security configured: bonding enabled, no PIN");
  
  NimBLEScan* pScan = NimBLEDevice::getScan();
  pScan->setAdvertisedDeviceCallbacks(new AdvCallbacks());
  pScan->setActiveScan(true);
  pScan->setInterval(100);
  pScan->setWindow(99);
  
  while(true) {
    if(!g_connected) {
      g_targetDevice = nullptr;
      Serial.println("Scanning for HID devices (10s)...");
      pScan->start(10, false);
      
      if(g_targetDevice) {
        Serial.println(">>> Found target, connecting...");
        
        g_client = NimBLEDevice::createClient();
        g_client->setClientCallbacks(new ClientCallbacks());
        g_client->setConnectTimeout(5);
        
        if(g_client->connect(g_targetDevice)) {
          Serial.println(">>> connect() returned true, discovering services...");
          
          // Wait a bit for connection to stabilize
          delay(500);
          
          // Initiate pairing/encryption
          Serial.println(">>> Initiating secure connection...");
          g_client->secureConnection();
          delay(1000); // Wait for pairing to complete
          
          NimBLERemoteService* pService = g_client->getService(NimBLEUUID((uint16_t)0x1812));
          if(pService) {
            Serial.println(">>> Found HID service");
            
            std::vector<NimBLERemoteCharacteristic*>* pCharacteristics = pService->getCharacteristics(true);
            Serial.print(">>> Characteristics: "); Serial.println(pCharacteristics->size());
            
            for(auto pChar : *pCharacteristics) {
              Serial.print("  ");
              Serial.print(pChar->getUUID().toString().c_str());
              Serial.print(" - R:");
              Serial.print(pChar->canRead() ? "Y" : "N");
              Serial.print(" W:");
              Serial.print(pChar->canWrite() ? "Y" : "N");
              Serial.print(" N:");
              Serial.println(pChar->canNotify() ? "Y" : "N");
              
              // Subscribe to Input Report (0x2A4D) with notify
              if(pChar->getUUID() == NimBLEUUID((uint16_t)0x2A4D) && pChar->canNotify()) {
                Serial.println("    -> Subscribing to Input Report...");
                if(pChar->subscribe(true, notifyCallback)) {
                  Serial.println("    -> Subscribe SUCCESS!");
                } else {
                  Serial.println("    -> Subscribe FAILED!");
                }
              }
            }
          } else {
            Serial.println(">>> HID service NOT found!");
          }
        } else {
          Serial.println(">>> connect() returned FALSE!");
        }
      } else {
        Serial.println(">>> No HID device found, retrying...");
      }
      
      delay(2000);
    } else {
      // Connected, just wait
      delay(1000);
      Serial.print(".");
    }
  }
}
