#include "hw/lasers.h"

#include "config.h"

namespace {
const uint8_t kXshut[3] = {pins::XSHUT_LEFT, pins::XSHUT_FRONT, pins::XSHUT_RIGHT};
const uint8_t kAddr[3] = {pins::ADDR_LEFT, pins::ADDR_FRONT, pins::ADDR_RIGHT};
}  // namespace

bool Lasers::begin() {
  for (int i = 0; i < 3; ++i) {
    pinMode(kXshut[i], OUTPUT);
    digitalWrite(kXshut[i], LOW);
  }
  delay(10);
  bool all = true;
  for (int i = 0; i < 3; ++i) {
    // El modulo tiene pull-up propio a 2.8 V: soltar el pin lo enciende sin meterle 3.3 V.
    pinMode(kXshut[i], INPUT);
    delay(10);
    VL53L0X& s = s_[i];
    s.setTimeout(30);
    if (!s.init()) {
      ok_[i] = false;
      all = false;
      continue;
    }
    s.setAddress(kAddr[i]);
    // 0.25 MCPS (valor de fabrica): con los laterales a 45 grados el eco es mas debil que de frente.
    s.setSignalRateLimit(0.25f);
    s.setMeasurementTimingBudget(20000);
    s.startContinuous(0);
    ok_[i] = true;
  }
  return all;
}

bool Lasers::poll(int i, int& mm) {
  if (!ok_[i]) return false;
  VL53L0X& s = s_[i];
  if ((s.readReg(VL53L0X::RESULT_INTERRUPT_STATUS) & 0x07) == 0) return false;
  // Estado de la medida (bits 6..3): 11 = valida. Senal debil, sigma alta o fase = lectura no fiable.
  const uint8_t status = (s.readReg(VL53L0X::RESULT_RANGE_STATUS) >> 3) & 0x0F;
  const uint16_t v = s.readRangeContinuousMillimeters();  // ya lista: no espera; borra la interrupcion
  mm = (s.timeoutOccurred() || status != 11 || v >= 8000) ? -1 : static_cast<int>(v);
  return true;
}

int Lasers::read(int i) {
  const uint32_t t0 = millis();
  int mm = -1;
  while (millis() - t0 < 100) {
    if (poll(i, mm)) return mm;
  }
  return -1;
}
