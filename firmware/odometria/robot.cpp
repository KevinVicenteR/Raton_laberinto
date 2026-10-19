#include "seguridad/robot_hw.h"
#include "comunicacion/telemetry.h"
#include "config.h"
#include "odometria/motion_progress.h"

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

