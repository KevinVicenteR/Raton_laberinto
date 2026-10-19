#include "hw/telemetry.h"

#include <ArduinoBLE.h>

#include "config.h"
#include "raton/frame.h"
#include "raton/line_queue.h"

namespace telemetry {

namespace {

class Telemetry {
 public:
  static Telemetry& instance() {
    static Telemetry value;
    return value;
  }

 private:
  BLEService uart{"6E400001-B5A3-F393-E0A9-E50E24DCCA9E"};
  BLECharacteristic rx{"6E400002-B5A3-F393-E0A9-E50E24DCCA9E", BLEWrite | BLEWriteWithoutResponse, 20};
  BLECharacteristic tx{"6E400003-B5A3-F393-E0A9-E50E24DCCA9E", BLENotify, 20};
  bool ready = false;
  uint32_t dropped = 0;
  uint16_t seq = 0;  // numero de la siguiente linea enviada (protocolo de frame.h)

  char inbuf[64];
  int inHead = 0;
  int inTail = 0;

  // Colas de salida (el control no espera nunca al Bluetooth). Guardan las lineas SIN numerar; el
  // numero y el CRC se ponen al sacarlas para enviar, asi la numeracion sigue el orden real de envio y la
  // web cuenta bien las perdidas. La prioritaria (ACK, modo, celdas, decisiones, avisos) sale antes que
  // la normal (estado y OBS). Si no hay sitio se tira la linea mas vieja todavia sin empezar a enviar.
  char normalBuf[1024];
  char priorityBuf[768];
  raton::LineQueue normal{normalBuf, sizeof(normalBuf)};
  raton::LineQueue priority{priorityBuf, sizeof(priorityBuf)};
  // Linea que se esta enviando (ya numerada) y trozo en vuelo (se reenvia hasta que se acepta).
  char active[208];
  int activeLen = 0;
  int activePos = 0;
  uint8_t chunk[20];
  int chunkLen = 0;
  bool sending() { return chunkLen > 0 || activePos < activeLen; }

  static void onWrite(BLEDevice, BLECharacteristic c) { instance().receive(c); }

  void receive(BLECharacteristic c) {
    const uint8_t* d = c.value();
    for (int i = 0; i < c.valueLength(); ++i) {
      const int nx = (inHead + 1) % static_cast<int>(sizeof(inbuf));
      if (nx == inTail) break;  // lleno
      inbuf[inHead] = static_cast<char>(d[i]);
      inHead = nx;
    }
  }

 public:
  bool begin() {
    if (!BLE.begin()) return false;
    BLE.setLocalName(wiring::BLE_NAME);
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
    if (!BLE.connected() || !tx.subscribed()) {
      // Sin nadie escuchando las notificaciones: no se envia nada (se perderia sin aviso).
      if (!BLE.connected()) {
        normal.clear();
        priority.clear();
        activeLen = activePos = chunkLen = 0;
      }
      return;
    }
    if (chunkLen == 0) {
      if (activePos >= activeLen) {
        // Siguiente linea entera: primero la prioritaria. Se numera aqui, en orden de envio.
        char raw[192];
        raton::LineQueue& q = !priority.empty() ? priority : normal;
        if (q.pop(raw, sizeof(raw)) < 0) return;
        const size_t n = raton::frameLine(seq++, raw, active, sizeof(active) - 1);
        if (!n) { ++dropped; return; }
        active[n] = '\n';
        activeLen = static_cast<int>(n) + 1;
        activePos = 0;
      }
      while (chunkLen < static_cast<int>(sizeof(chunk)) && activePos < activeLen) chunk[chunkLen++] = active[activePos++];
    }
    // Solo se da por enviado si el Bluetooth lo acepta; si no, se reintenta el mismo trozo.
    if (tx.writeValue(chunk, chunkLen)) chunkLen = 0;
  }

  void println(const char* raw, bool important) {
    if (!ready || !BLE.connected()) return;
    const int len = static_cast<int>(strlen(raw));
    if (len + 1 > 190) {
      ++dropped;
      return;
    }
    const int lost = (important ? priority : normal).push(raw, len);
    dropped += lost < 0 ? 1 : lost;
  }

  void println(const char* raw) {
    // Las lineas de estado y observacion van a la cola normal; el resto (ordenes, decisiones, avisos)
    // es importante.
    const bool routine = strncmp(raw, "OBS ", 4) == 0 || strncmp(raw, "MOTION ", 7) == 0;
    println(raw, !routine);
  }

  uint32_t droppedLines() { return dropped; }
  int pendingPriorityBytes() { return priority.used(); }

  void flush(uint32_t ms) {
    const uint32_t t0 = millis();
    while (ready && BLE.connected() && (!priority.empty() || !normal.empty() || sending()) && millis() - t0 < ms) {
      poll();
      delay(3);
    }
    const uint32_t t1 = millis();
    while (ready && millis() - t1 < 60) {  // que el controlador BLE llegue a mandar la ultima notificacion
      BLE.poll();
      delay(3);
    }
  }

  int read() {
    if (inHead == inTail) return -1;
    const char c = inbuf[inTail];
    inTail = (inTail + 1) % static_cast<int>(sizeof(inbuf));
    return c;
  }

};
}  // namespace

bool begin() { return Telemetry::instance().begin(); }
void poll() { Telemetry::instance().poll(); }
void println(const char* raw, bool important) { Telemetry::instance().println(raw, important); }
void println(const char* raw) { Telemetry::instance().println(raw); }
uint32_t droppedLines() { return Telemetry::instance().droppedLines(); }
int pendingPriorityBytes() { return Telemetry::instance().pendingPriorityBytes(); }
void flush(uint32_t ms) { Telemetry::instance().flush(ms); }
int read() { return Telemetry::instance().read(); }

}  // namespace telemetry
