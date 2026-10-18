#include "robot/robot_hw.h"

#include "comms/telemetry.h"
#include "config.h"

namespace {

raton::GeometryParams geometryParams() {
  raton::GeometryParams p;
  p.sideSensorAheadMm = cfg::SIDE_SENSOR_AHEAD_MM;
  p.sideSensorLateralMm = cfg::SIDE_SENSOR_LATERAL_MM;
  p.frontSensorAheadMm = cfg::FRONT_SENSOR_AHEAD_MM;
  p.frontMarginMm = cfg::FRONT_MARGIN_MM;
  p.bodyWidthMm = cfg::BODY_WIDTH_MM;
  p.wallMm = cfg::WALL_MM;
  p.sideWallFactor = cfg::SIDE_WALL_FACTOR;
  return p;
}

}  // namespace

RobotHW::RobotHW(Motors& motors, Encoders& encoders, Lasers& lasers, Gyro& gyro, Display& display)
    : motors_(motors), encoders_(encoders), lasers_(lasers), gyro_(gyro), display_(display), commands_(*this),
      geom_(geometryParams(), cfg::DEFAULT_CORRIDOR_MM, cfg::CELL_MM, cfg::SIDE_SENSOR_ANGLE_DEG,
            cfg::FRONT_DEADZONE_MM) {}

// ---------------------------------------------------------------- ciclo de control

void RobotHW::waitPeriod(uint32_t& next) {
  const uint32_t now = micros();
  if (lastPeriodUs_ != 0 && now - lastPeriodUs_ > maxLagUs_) maxLagUs_ = now - lastPeriodUs_;
  // Si el ciclo se retraso mucho, no intentar recuperar ciclos perdidos de golpe.
  if (static_cast<int32_t>(now - next) > static_cast<int32_t>(4 * cfg::CONTROL_PERIOD_US)) next = now;
  while (static_cast<int32_t>(micros() - next) < 0) {
    lasers_.update();
    gyro_.update();
  }
  service();
  laserSamples_ = lasers_.sampleCount(Lasers::LEFT) + lasers_.sampleCount(Lasers::FRONT) +
                  lasers_.sampleCount(Lasers::RIGHT);
  sendTelemetry();
  lastPeriodUs_ = micros();
  next += cfg::CONTROL_PERIOD_US;
}

void RobotHW::service() {
  telemetry::poll();
  commands_.poll();
  updateOdometry();
}

void RobotHW::idle(uint32_t ms) {
  const uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    lasers_.update();
    gyro_.update(true);
    service();
    sendTelemetry();
  }
}

bool RobotHW::takeStart() {
  const bool s = requests_.start;
  requests_.start = false;
  return s;
}

int RobotHW::takeMode() {
  const int m = requests_.mode;
  requests_.mode = 0;
  return m;
}

// ---------------------------------------------------------------- odometria y memoria del modo 5

void RobotHW::resetOdometry() {
  odoX_ = 0;
  odoY_ = 0;
  odoYaw0_ = gyro_.totalYaw();
  odoLastA_ = 0;
  memset(visits_, 0, sizeof(visits_));
  memset(exits_, 0, sizeof(exits_));
  pendingAdvance_ = 0;
  cellX_ = cellY_ = -1;
  odoL_ = encoders_.leftTotal();
  odoR_ = encoders_.rightTotal();
}

void RobotHW::updateOdometry() {
  const long l = encoders_.leftTotal();
  const long r = encoders_.rightTotal();
  const float ds = 0.5f * ((l - odoL_) + (r - odoR_)) * cfg::MM_PER_COUNT;
  odoL_ = l;
  odoR_ = r;
  odoDist_ += ds;
  for (int i = 0; i < 3; ++i) {
    if (lasers_.mm(static_cast<Lasers::Id>(i)) >= 0) lastValidDist_[i] = odoDist_;
  }
  // El rumbo sale del giroscopio (no patina); 0 = mirando hacia +Y al empezar. Se integra con el
  // rumbo medio del paso para que los giros en marcha no deformen el mapa.
  const float a = (gyro_.totalYaw() - odoYaw0_) / 57.2958f;
  const float am = 0.5f * (a + odoLastA_);
  odoLastA_ = a;
  odoX_ -= ds * sinf(am);
  odoY_ += ds * cosf(am);
  const int cx = cellOf(odoX_), cy = cellOf(odoY_ + 0.5f * geom_.corridorMm());
  if (cx != cellX_ || cy != cellY_) {
    cellX_ = cx;
    cellY_ = cy;
    if (cx >= 0 && cy >= 0 && cx < kGrid && cy < kGrid && visits_[cx][cy] < 255) visits_[cx][cy]++;
  }
}

int RobotHW::cellOf(float v) const {
  const float pitch = geom_.corridorMm() + 15.0f;  // pasillo + pared
  return static_cast<int>(floorf(v / pitch + 0.5f)) + kGrid / 2;
}

int RobotHW::worldDir(float turnDeg) const {
  const float h = (gyro_.totalYaw() - odoYaw0_) + turnDeg;
  return ((static_cast<int>(lroundf(h / 90.0f)) % 4) + 4) % 4;
}

void RobotHW::setGoal(float x, float y) {
  goalX_mm_ = x;
  goalY_mm_ = y;
  goalSet_ = true;
  goalReached_ = false;
}

float RobotHW::goalAlignment(float turnDeg) const {
  if (!goalSet_) return 0.0f;
  const float gx = goalX_mm_ - odoX_, gy = goalY_mm_ - odoY_;
  const float len = hypotf(gx, gy);
  if (len < 1.0f) return 1.0f;
  const int d = worldDir(turnDeg);
  const float dx[4] = {0, -1, 0, 1}, dy[4] = {1, 0, -1, 0};
  return (dx[d] * gx + dy[d] * gy) / len;
}

int RobotHW::exitsToward(float turnDeg) const {
  if (cellX_ < 0 || cellX_ >= kGrid || cellY_ < 0 || cellY_ >= kGrid) return 0;
  return exits_[cellX_][cellY_][worldDir(turnDeg)];
}

void RobotHW::markExit(float turnDeg) {
  if (cellX_ < 0 || cellX_ >= kGrid || cellY_ < 0 || cellY_ >= kGrid) return;
  uint8_t& e = exits_[cellX_][cellY_][worldDir(turnDeg)];
  if (e < 255) e++;
}

int RobotHW::visitsToward(float turnDeg) const {
  if (cellX_ < 0) return 0;
  // Direccion en la pista: 0 = norte (+y), 1 = oeste, 2 = sur, 3 = este (angulos positivos a la izquierda).
  const int d = worldDir(turnDeg);
  const int dx[4] = {0, -1, 0, 1}, dy[4] = {1, 0, -1, 0};
  const int x = cellX_ + dx[d], y = cellY_ + dy[d];
  if (x < 0 || y < 0 || x >= kGrid || y >= kGrid) return 0;
  return visits_[x][y];
}

// ---------------------------------------------------------------- rumbo

void RobotHW::resetHeading() {
  gyro_.update();
  gyro_.resetYaw();
  heading_ = 0;
  headingBase_ = 0;
  lastGyroYaw_ = 0;
}

void RobotHW::resetHeadingToGrid() {
  resetHeading();
  // Con cuadricula, el rumbo cero es el de la pista, no hacia donde mira ahora.
  if (gridOn_) heading_ = headingBase_ = gridError();
}

void RobotHW::setGrid(bool on) {
  gridOn_ = on;
  gridYaw_ = gyro_.totalYaw();
}

float RobotHW::gridError() const {
  // Cuanto esta girado respecto al rumbo de la cuadricula mas cercano (multiplo de 90).
  float e = fmodf(gyro_.totalYaw() - gridYaw_, 90.0f);
  if (e > 45.0f) e -= 90.0f;
  if (e < -45.0f) e += 90.0f;
  return e;
}

float RobotHW::gridTurnError(float degrees) {
  // Avanza el norte de la cuadricula y devuelve lo que falta para llegar a ese rumbo exacto.
  gridYaw_ += degrees;
  float e = fmodf(gridYaw_ - gyro_.totalYaw(), 360.0f);
  if (e > 180.0f) e -= 360.0f;
  if (e < -180.0f) e += 360.0f;
  return e;
}

void RobotHW::updateHeading() {
  // Filtro complementario: el giroscopio manda a corto plazo; los encoders (sin deriva, pero con
  // patinazos) corrigen poco a poco. Requiere encoders_.reset() junto con resetHeading().
  const float g = gyro_.yaw();
  heading_ += g - lastGyroYaw_;
  lastGyroYaw_ = g;
  const float diff = (encoders_.right() - encoders_.left()) * cfg::MM_PER_COUNT;
  const float encYaw = headingBase_ + diff / trackWidth_ * 57.2958f;
  heading_ += cfg::ENC_YAW_WEIGHT * (encYaw - heading_);
}

// ---------------------------------------------------------------- telemetria de estado

void RobotHW::recordMotion(const char* phase, float requested, const raton::MotionResult& result) {
  char line[384];
  snprintf(line, sizeof(line),
           "MOTION t=%lu phase=%s req=%d actual=%d error_cdeg=%d completed=%d reason=%d enc=%ld,%ld raw=%d,%d,%d "
           "filtered=%d,%d,%d age=%lu,%lu,%lu conf=%d,%d,%d",
           static_cast<unsigned long>(millis()), phase, static_cast<int>(requested), static_cast<int>(result.actualMm),
           static_cast<int>(result.angularErrorDeg * 100), result.completed, static_cast<int>(result.reason),
           encoders_.leftTotal(), encoders_.rightTotal(), lasers_.acquiredMm(Lasers::LEFT),
           lasers_.acquiredMm(Lasers::FRONT), lasers_.acquiredMm(Lasers::RIGHT), lasers_.mm(Lasers::LEFT),
           lasers_.mm(Lasers::FRONT), lasers_.mm(Lasers::RIGHT),
           static_cast<unsigned long>(lasers_.lastValidAgeMs(Lasers::LEFT)),
           static_cast<unsigned long>(lasers_.lastValidAgeMs(Lasers::FRONT)),
           static_cast<unsigned long>(lasers_.lastValidAgeMs(Lasers::RIGHT)),
           static_cast<int>(100 * lasers_.confidence(Lasers::LEFT)), static_cast<int>(100 * lasers_.confidence(Lasers::FRONT)),
           static_cast<int>(100 * lasers_.confidence(Lasers::RIGHT)));
  telemetry::println(line);
}

void RobotHW::sendTelemetry() {
  if (millis() - lastTelemetry_ < cfg::TELEMETRY_PERIOD_MS) return;
  lastTelemetry_ = millis();
  if (millis() - lastObservationTelemetry_ >= cfg::OBS_PERIOD_MS) {
    lastObservationTelemetry_ = millis();
    char sample[256];
    snprintf(sample, sizeof(sample),
             "OBS t=%lu raw=%d,%d,%d filtered=%d,%d,%d age=%lu,%lu,%lu conf=%d,%d,%d enc=%ld,%ld yaw=%d phase=%s drops=%lu",
             static_cast<unsigned long>(millis()), lasers_.acquiredMm(Lasers::LEFT), lasers_.acquiredMm(Lasers::FRONT),
             lasers_.acquiredMm(Lasers::RIGHT), lasers_.mm(Lasers::LEFT), lasers_.mm(Lasers::FRONT),
             lasers_.mm(Lasers::RIGHT), static_cast<unsigned long>(lasers_.lastValidAgeMs(Lasers::LEFT)),
             static_cast<unsigned long>(lasers_.lastValidAgeMs(Lasers::FRONT)),
             static_cast<unsigned long>(lasers_.lastValidAgeMs(Lasers::RIGHT)),
             static_cast<int>(100 * lasers_.confidence(Lasers::LEFT)),
             static_cast<int>(100 * lasers_.confidence(Lasers::FRONT)),
             static_cast<int>(100 * lasers_.confidence(Lasers::RIGHT)), encoders_.leftTotal(), encoders_.rightTotal(),
             static_cast<int>(gyro_.totalYaw() * 100), state_, static_cast<unsigned long>(telemetry::droppedLines()));
    telemetry::printStatus(sample);
  }
  char line[160];
  const int a = static_cast<int>(gyro_.totalYaw() - odoYaw0_);
  // q = lecturas SIN filtrar (para el mapa: el filtro mantiene valores viejos cuando el laser no mide).
  snprintf(line, sizeof(line), "%s I%d F%d D%d v%d r%d p%d x%d y%d a%d t%d q%d,%d,%d c%d,%d,%d", state_,
           lasers_.mm(Lasers::LEFT), lasers_.mm(Lasers::FRONT), lasers_.mm(Lasers::RIGHT), static_cast<int>(lastSpeed_),
           static_cast<int>(heading_), static_cast<int>(geom_.corridorMm()), static_cast<int>(odoX_),
           static_cast<int>(odoY_), a, static_cast<int>(maxLagUs_ / 1000), lasers_.rawMm(Lasers::LEFT),
           lasers_.rawMm(Lasers::FRONT), lasers_.rawMm(Lasers::RIGHT),
           static_cast<int>(lasers_.confidence(Lasers::LEFT) * 100), static_cast<int>(lasers_.confidence(Lasers::FRONT) * 100),
           static_cast<int>(lasers_.confidence(Lasers::RIGHT) * 100));
  maxLagUs_ = 0;
  telemetry::printStatus(line);
}
