#pragma once
#include <stdint.h>
enum { BLEWrite = 1, BLEWriteWithoutResponse = 2, BLENotify = 4, BLEWritten = 1 };
struct BLEDevice {};
struct BLECharacteristic {
  BLECharacteristic(const char*, int, int) {}
  const uint8_t* value() { return nullptr; }
  int valueLength() { return 0; }
  bool subscribed() { return false; }
  int writeValue(const uint8_t*, int) { return 1; }
  void setEventHandler(int, void (*)(BLEDevice, BLECharacteristic)) {}
};
struct BLEService {
  explicit BLEService(const char*) {}
  void addCharacteristic(BLECharacteristic&) {}
};
struct BLELocalDevice {
  int begin() { return 1; }
  void setLocalName(const char*) {}
  void setConnectionInterval(int, int) {}
  void setAdvertisedService(BLEService&) {}
  void addService(BLEService&) {}
  void advertise() {}
  void poll() {}
  bool connected() { return false; }
};
extern BLELocalDevice BLE;
