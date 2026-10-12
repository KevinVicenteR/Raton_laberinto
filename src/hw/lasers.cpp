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
    raw_[i] = (s.timeoutOccurred() || v >= 2000) ? -1 : static_cast<int>(v);
    filter(i, raw_[i]);
    samples_[i]++;
  }
}

void Lasers::filter(int i, int z) {
  // Sin medida: se mantiene la estimacion hasta LASER_MAX_MISS lecturas seguidas (ignora -1 sueltos).
  if (z < 0) {
    if (++misses_[i] >= cfg::LASER_MAX_MISS) {
      kValid_[i] = false;
      mm_[i] = -1;
    }
    return;
  }
  misses_[i] = 0;

  const uint32_t now = micros();
  float* p = kp_[i];
  const float zf = static_cast<float>(z);
  auto restart = [&]() {
    kd_[i] = zf;
    kv_[i] = 0;
    p[0] = cfg::LASER_KF_R;
    p[1] = p[2] = 0;
    p[3] = 300.0f * 300.0f;  // velocidad desconocida al empezar
    jumps_[i] = 0;
  };

  if (!kValid_[i]) {
    restart();
    kValid_[i] = true;
  } else {
    float dt = (now - kt_[i]) * 1e-6f;
    dt = dt < 0.005f ? 0.005f : (dt > 0.1f ? 0.1f : dt);

    // Prediccion con velocidad constante; Q de una aceleracion aleatoria LASER_KF_ACCEL.
    kd_[i] += kv_[i] * dt;
    const float a2 = cfg::LASER_KF_ACCEL * cfg::LASER_KF_ACCEL;
    const float dt2 = dt * dt;
    const float p00 = p[0] + dt * (p[1] + p[2]) + dt2 * p[3] + 0.25f * dt2 * dt2 * a2;
    const float p01 = p[1] + dt * p[3] + 0.5f * dt2 * dt * a2;
    const float p10 = p[2] + dt * p[3] + 0.5f * dt2 * dt * a2;
    const float p11 = p[3] + dt2 * a2;

    const float innovation = zf - kd_[i];
    if (fabsf(innovation) > cfg::LASER_JUMP_MM) {
      // Salto grande: una lectura suelta se descarta; si se repite es real (hueco o pared nueva).
      p[0] = p00;
      p[1] = p01;
      p[2] = p10;
      p[3] = p11;
      if (++jumps_[i] >= cfg::LASER_JUMP_CONFIRM) restart();
    } else {
      jumps_[i] = 0;
      const float s = p00 + cfg::LASER_KF_R;
      const float k0 = p00 / s;
      const float k1 = p10 / s;
      kd_[i] += k0 * innovation;
      kv_[i] += k1 * innovation;
      p[0] = (1 - k0) * p00;
      p[1] = (1 - k0) * p01;
      p[2] = p10 - k1 * p00;
      p[3] = p11 - k1 * p01;
    }
  }
  kt_[i] = now;
  mm_[i] = kd_[i] < 0 ? 0 : static_cast<int>(kd_[i] + 0.5f);
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
