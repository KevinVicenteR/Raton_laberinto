#include "seguridad/robot_hw.h"
#include "comunicacion/telemetry.h"
#include "config.h"
#include "odometria/motion_progress.h"

int RobotHW::frontMm() {
  const int f = lasers_.mm(Lasers::FRONT);
  if (f >= 0) {
    frontMemMm_ = f;
    frontMemDist_ = odoDist_;
    return f;
  }
  if (frontMemMm_ >= 0 && frontMemMm_ < cfg::FRONT_MEMORY_MM) {
    const float predicted = frontMemMm_ - (odoDist_ - frontMemDist_);
    if (predicted < cfg::FRONT_MEMORY_MM) return predicted > 0 ? static_cast<int>(predicted) : 0;
  }
  frontMemMm_ = -1;
  return -1;
}

Walls RobotHW::readWalls() {
  waitFresh(2);
  auto wall = [](int mm, int threshold) { return mm >= 0 && mm < threshold; };
  return Walls{wall(sideMm(Lasers::LEFT), tuning_.sideWall), wall(frontMm(), cfg::FRONT_WALL_MM),
               wall(sideMm(Lasers::RIGHT), tuning_.sideWall)};
}

Walls RobotHW::senseCell() {
  int best[3] = {-1, -1, -1};
  for (int k = 0; k < 5; ++k) {
    waitFresh(1);
    for (int i = 0; i < 3; ++i) {
      const int v = lasers_.mm(static_cast<Lasers::Id>(i));
      if (v >= 0 && (best[i] < 0 || v < best[i])) best[i] = v;
    }
  }
  const int threshold = geom_.cellFrontWall();
  auto wall = [&](int i) {
    if (best[i] >= 0) return best[i] < threshold;
    // Ninguna lectura: si la ultima valida era muy corta y fue hace menos de 4 cm de recorrido, esta
    // pegado a la pared (el VL53L0X no mide tan cerca). En un hueco de verdad fue hace casi una celda.
    const Lasers::Id id = static_cast<Lasers::Id>(i);
    return lasers_.lastValidMm(id) < cfg::SIDE_CLOSE_MM && fabsf(odoDist_ - lastValidDist_[i]) < 40.0f;
  };
  return Walls{wall(Lasers::LEFT), wall(Lasers::FRONT), wall(Lasers::RIGHT)};
}

bool RobotHW::cellSides(int i, bool& left, bool& right) const {
  if (i < 0 || i >= numCellSides_) return false;
  left = cellSideL_[i];
  right = cellSideR_[i];
  return true;
}

void RobotHW::waitFresh(int samples) {
  uint32_t start[3];
  for (int i = 0; i < 3; ++i) start[i] = lasers_.sampleCount(static_cast<Lasers::Id>(i));
  const uint32_t t0 = millis();
  while (!aborted_ && millis() - t0 < 200) {
    idle(1);
    bool done = true;
    for (int i = 0; i < 3; ++i)
      if (lasers_.sampleCount(static_cast<Lasers::Id>(i)) - start[i] < static_cast<uint32_t>(samples)) done = false;
    if (done) break;
  }
}

bool RobotHW::frontWallStopped() {
  // Dos entregas frontales independientes ya confirmadas durante el avance.
  // No volver a adquirir los tres sensores al decidir tras cada celda.
  const int front = cachedCellFront_ ? cachedCellFrontMm_ : stableFrontMm();
  return front >= 0 && front < geom_.frontTurnMm() + static_cast<int>(cfg::FRONT_WALL_FACTOR * geom_.cellMm());
}

void RobotHW::observeCellSides(float pos, int cells, float cellMm, float already,
                               float& target, raton::CellObserver*& observer, raton::CellWindow& window) {
    if (numCellSides_ < cells && numCellSides_ < kMaxCellSides) {
      const float at = (numCellSides_ + 1) * cellMm - already - geom_.sideLookAhead();
      if (pos >= at - kSideWindowMm) {
        const int l = sideMm(Lasers::LEFT), r = sideMm(Lasers::RIGHT);
        if (laserSamples_ != window.winSample) {  // solo lecturas nuevas
          window.winSample = laserSamples_;
          if (l >= 0 && window.nWinL < 24) window.winL[window.nWinL++] = l;
          if (r >= 0 && window.nWinR < 24) window.winR[window.nWinR++] = r;
        }
        window.windowSeen = true;
      }
      if (window.windowSeen && pos >= at + kSideWindowMm) {
        const int mL = raton::medianOf(window.winL, window.nWinL), mR = raton::medianOf(window.winR, window.nWinR);
        cellSideL_[numCellSides_] = mL >= 0 && mL < geom_.cellSideWall();
        cellSideR_[numCellSides_] = mR >= 0 && mR < geom_.cellSideWall();
        numCellSides_++;
        window.nWinL = window.nWinR = 0;
        window.windowSeen = false;
        // Exploracion en marcha: el explorador decide con las paredes recien leidas si sigue recto o se
        // para en esta celda (para girar). Asi no se para en cada celda.
        if (observer && !observer->onCellSides(numCellSides_ - 1, cellSideL_[numCellSides_ - 1],
                                               cellSideR_[numCellSides_ - 1], frontMm())) {
          const float stopAt = numCellSides_ * cellMm - already;
          if (stopAt < target) target = fmaxf(stopAt, pos);
          observer = nullptr;  // ya decidido: el resto del tramo es frenar
        }
      }
    }
}

void RobotHW::confirmCellFront(int frontNow, uint32_t& frontSample, int& frontPrev, int& frontConfirmed) {
    if (lasers_.sampleCount(Lasers::FRONT) != frontSample) {
      frontSample = lasers_.sampleCount(Lasers::FRONT);
      frontConfirmed = (frontNow >= 0 && frontPrev >= 0 && abs(frontNow - frontPrev) <= 30) ? frontNow
                       : (frontNow < 0 ? -1 : frontConfirmed);
      frontPrev = frontNow;
    }
}
