#include "lasers.h"

#include "../config.h"

namespace {
const uint8_t kXshut[3] = {cfg::PIN_XSHUT_IZQ, cfg::PIN_XSHUT_FRO, cfg::PIN_XSHUT_DER};
const uint8_t kAddress[3] = {cfg::I2C_LASER_IZQ, cfg::I2C_LASER_FRO, cfg::I2C_LASER_DER};
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
    VL53L0X& s = sensors_[i];
    s.setTimeout(50);
    if (!s.init()) {
      ok_[i] = false;
      all = false;
      continue;
    }
    s.setAddress(kAddress[i]);
    s.setMeasurementTimingBudget(20000);
    s.startContinuous();
    ok_[i] = true;
  }
  return all;
}

void Lasers::update() {
  for (int i = 0; i < 3; ++i) {
    if (!ok_[i]) continue;
    VL53L0X& s = sensors_[i];
    // Solo lee si ya hay medida lista; asi readRangeContinuousMillimeters no bloquea.
    if ((s.readReg(VL53L0X::RESULT_INTERRUPT_STATUS) & 0x07) == 0) continue;
    const uint16_t v = s.readRangeContinuousMillimeters();
    mm_[i] = (s.timeoutOccurred() || v >= 2000) ? -1 : static_cast<int>(v);
    samples_[i]++;
  }
}

void Lasers::waitFresh(int n) {
  uint32_t start[3];
  for (int i = 0; i < 3; ++i) start[i] = samples_[i];
  const uint32_t t0 = millis();
  while (millis() - t0 < 200) {
    update();
    bool done = true;
    for (int i = 0; i < 3; ++i) {
      if (ok_[i] && samples_[i] - start[i] < static_cast<uint32_t>(n)) done = false;
    }
    if (done) return;
  }
}
