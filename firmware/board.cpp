#include "board.h"

#include <Arduino.h>

#include "hw/telemetry.h"
#include "storage.h"

namespace {
const float kPi = 3.14159265f;
}

void Board::setMotors(float left, float right) {
  dutyL_ = left;
  dutyR_ = right;
  motors_.set(left, right);
}

const raton::Sensors& Board::tick() {
  const uint32_t period = static_cast<uint32_t>(p_.dt * 1e6f);
  if (!started_) {
    started_ = true;
    t0_ = micros();
    next_ = t0_;
  }
  // Espera al siguiente periodo atendiendo comunicaciones.
  // Tiempo de calculo del ciclo anterior (desde que se devolvieron los sensores hasta ahora).
  const float busy = static_cast<float>(micros() - lastReturn_);
  if (lastReturn_ && busy > maxLoad_) maxLoad_ = busy;
  next_ += period;
  const int32_t slack = static_cast<int32_t>(next_ - micros());
  if (slack < -static_cast<int32_t>(period)) {
    ++late_;
    next_ = micros();  // muy tarde (p. ej. escribiendo la flash): no intentar recuperar ciclos perdidos
  }
  // Bluetooth y Serie en cada ciclo aunque no sobre tiempo (si no, con el I2C lento nunca se atienden y la
  // web no recibe ni manda nada); y otra vez mientras sobre tiempo.
  if (service) service();
  while (static_cast<int32_t>(next_ - micros()) > 600) {
    if (service) service();
  }
  while (static_cast<int32_t>(next_ - micros()) > 0) {
  }
  const uint32_t now = micros();
  s_.t = (now - t0_) * 1e-6f;

  // Encoders
  const long l = enc_.left(), r = enc_.right();
  const float mmPerCount = kPi * p_.wheelDiamMm / p_.countsPerRev;
  s_.encLeftMm = l * mmPerCount;
  s_.encRightMm = r * mmPerCount;

  // Giroscopio (y sesgo con el raton quieto)
  // El giroscopio da una muestra cada 5 ms: preguntar cada 2 ciclos (4 ms) basta y ahorra I2C.
  s_.gyroFresh = (++imuTick_ & 1) == 0 && imu_.poll(s_.t);
  if (s_.gyroFresh) s_.gyroDeg = imu_.yawDeg();
  const bool still = dutyL_ == 0 && dutyR_ == 0 && l == lastL_ && r == lastR_;
  stillT_ = still ? stillT_ + p_.dt : 0;
  imu_.trackBias(stillT_ > 0.3f);
  lastL_ = l;
  lastR_ = r;

  // Un laser por ciclo, por turnos (cada uno mide a ~50 Hz; asi el ciclo no pasa de ~0.6 ms de I2C).
  raton::Range* rr[3] = {&s_.left, &s_.front, &s_.right};
  const int offs[3] = {p_.leftOffsetMm, p_.frontOffsetMm, p_.rightOffsetMm};
  for (int i = 0; i < 3; ++i) rr[i]->fresh = false;
  int mm;
  if (lasers_.poll(laser_, mm)) {
    if (mm >= 0) mm += offs[laser_];
    if (mm > p_.sensorMaxMm) mm = -1;
    rr[laser_]->mm = mm;
    rr[laser_]->fresh = true;
  }
  laser_ = (laser_ + 1) % 3;
  lastReturn_ = micros();
  return s_;
}

void Board::report(const char* line, bool important) {
  telemetry::println(line, important);
  if (important && Serial) Serial.println(line);
}

bool Board::saveMap(const raton::Maze& maze) {
  motors_.set(0, 0);
  return storage::saveMap(maze);
}
