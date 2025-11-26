// Header for separated BLE callback classes used by btle_hid
#pragma once

#include <NimBLEDevice.h>
#include "logging.h"

// Forward-declare notify callback implemented in btle_hid.cpp
void notifyCallback(NimBLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify);

// Externs for shared state defined in btle_hid.cpp
extern NimBLEClient* g_client;
extern NimBLEAdvertisedDevice* g_targetDevice;
extern volatile bool g_connected;

// Lightweight callback classes (definitions in btle_callbacks.cpp)
class ClientCallbacks : public NimBLEClientCallbacks {
public:
  void onConnect(NimBLEClient* pClient) override;
  void onDisconnect(NimBLEClient* pClient) override;
  void onAuthenticationComplete(ble_gap_conn_desc* desc) override;
  bool onConfirmPIN(uint32_t pin) override;
};

class AdvCallbacks : public NimBLEAdvertisedDeviceCallbacks {
public:
  void onResult(NimBLEAdvertisedDevice* advertisedDevice) override;
};
