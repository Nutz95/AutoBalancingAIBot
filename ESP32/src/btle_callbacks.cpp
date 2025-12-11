#include "btle_callbacks.h"
#include <NimBLEDevice.h>
#include "logging.h"
#include "motor_drivers/driver_manager.h"

// Implement externs (actual storage is in btle_hid.cpp)
// (Declarations are extern in header; storage is defined in btle_hid.cpp)

void ClientCallbacks::onConnect(NimBLEClient* pClient) {
  g_connected = true;
  LOG_PRINTLN(abbot::log::CHANNEL_BLE, ">>> BLE: Connected callback fired");
  pClient->updateConnParams(12, 24, 0, 60);
}

void ClientCallbacks::onDisconnect(NimBLEClient* pClient) {
  g_connected = false;
  LOG_PRINTLN(abbot::log::CHANNEL_BLE, ">>> BLE: Disconnected");
  // NOTE: Do NOT auto-disable motors on controller disconnect here.
  // Motor enable/disable should be controlled explicitly by user commands
  // (e.g. via serial/menu) to avoid unexpected motor cutouts during bench tests.
}

void ClientCallbacks::onAuthenticationComplete(ble_gap_conn_desc* desc) {
  LOG_PRINT(abbot::log::CHANNEL_BLE, ">>> AUTH complete - encrypted: ");
  LOG_PRINTLN(abbot::log::CHANNEL_BLE, desc->sec_state.encrypted ? "YES" : "NO");
  if (desc->sec_state.encrypted) {
    LOG_PRINTLN(abbot::log::CHANNEL_BLE, ">>> Link is now encrypted, data should flow!");
  }
}

bool ClientCallbacks::onConfirmPIN(uint32_t pin) {
  LOG_PRINT(abbot::log::CHANNEL_BLE, ">>> Confirm PIN: ");
  LOG_PRINTF(abbot::log::CHANNEL_BLE, "%u\n", pin);
  return true;
}

void AdvCallbacks::onResult(NimBLEAdvertisedDevice* advertisedDevice) {
  LOG_PRINTF(abbot::log::CHANNEL_BLE, "Found: %s [%s]\n", advertisedDevice->getName().c_str(), advertisedDevice->getAddress().toString().c_str());
  if (advertisedDevice->isAdvertisingService(NimBLEUUID((uint16_t)0x1812))) {
    LOG_PRINTLN(abbot::log::CHANNEL_BLE, " -> HID");
    g_targetDevice = advertisedDevice;
    NimBLEDevice::getScan()->stop();
  } else {
    LOG_PRINTLN(abbot::log::CHANNEL_BLE, "");
  }
}
