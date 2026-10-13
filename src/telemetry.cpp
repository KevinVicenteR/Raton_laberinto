#include "telemetry.h"

#include <ArduinoBLE.h>

#include "config.h"

namespace telemetry {

namespace {

BLEService uart("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
BLECharacteristic rx("6E400002-B5A3-F393-E0A9-E50E24DCCA9E", BLEWrite | BLEWriteWithoutResponse, 20);
BLECharacteristic tx("6E400003-B5A3-F393-E0A9-E50E24DCCA9E", BLENotify, 20);
bool ready = false;

char inbuf[64];
int inHead = 0;
int inTail = 0;

// Cola de salida: el control no espera nunca al Bluetooth.
char outbuf[1024];
int outHead = 0;
int outTail = 0;
int outUsed() { return (outHead - outTail + static_cast<int>(sizeof(outbuf))) % static_cast<int>(sizeof(outbuf)); }

void onWrite(BLEDevice, BLECharacteristic c) {
  const uint8_t* d = c.value();
  for (int i = 0; i < c.valueLength(); ++i) {
    const int nx = (inHead + 1) % static_cast<int>(sizeof(inbuf));
    if (nx == inTail) break;  // lleno
    inbuf[inHead] = static_cast<char>(d[i]);
    inHead = nx;
  }
}

}  // namespace

bool begin() {
  if (!BLE.begin()) return false;
  BLE.setLocalName(cfg::BLE_NAME);
  BLE.setConnectionInterval(6, 12);  // 7.5-15 ms: mas datos por segundo para el mapa
  BLE.setAdvertisedService(uart);
  uart.addCharacteristic(rx);
  uart.addCharacteristic(tx);
  BLE.addService(uart);
  rx.setEventHandler(BLEWritten, onWrite);
  BLE.advertise();
  ready = true;
  return true;
}

void poll() {
  if (!ready) return;
  BLE.poll();
  if (outHead == outTail) return;
  if (!BLE.connected()) {
    outHead = outTail = 0;
    return;
  }
  // Un trozo de hasta 20 bytes (tamano de una notificacion) por llamada.
  uint8_t buf[20];
  int k = 0;
  while (k < static_cast<int>(sizeof(buf)) && outTail != outHead) {
    buf[k++] = static_cast<uint8_t>(outbuf[outTail]);
    outTail = (outTail + 1) % static_cast<int>(sizeof(outbuf));
  }
  tx.writeValue(buf, k);
}

void println(const char* line) {
  if (!ready || !BLE.connected()) return;
  const int n = static_cast<int>(strlen(line)) + 1;  // con el salto de linea
  if (outUsed() + n >= static_cast<int>(sizeof(outbuf))) return;  // llena: se pierde esta linea
  for (int i = 0; i < n; ++i) {
    outbuf[outHead] = i < n - 1 ? line[i] : '\n';
    outHead = (outHead + 1) % static_cast<int>(sizeof(outbuf));
  }
}

int read() {
  if (inHead == inTail) return -1;
  const char c = inbuf[inTail];
  inTail = (inTail + 1) % static_cast<int>(sizeof(inbuf));
  return c;
}

}  // namespace telemetry
