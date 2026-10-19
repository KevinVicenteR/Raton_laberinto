#include "raton/navigator.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

namespace raton {

namespace {

const float kDeg = 57.2957795f;

float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

// Media vuelta hacia -180..180.
float wrap180(float a) {
  while (a > 180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

}  // namespace

Navigator::Navigator(Platform& io, const Params& p, Maze& maze) : io_(io), p_(p), maze_(maze), motion_(p) {}

void Navigator::computeSmoothGeometry() {
  // Curva de 90 grados a velocidad constante: rampa angular corta, tramo a w = v / R y rampa de salida.
  // Escalando w con v y la aceleracion angular con v^2 la forma es siempre la misma, asi que se calcula una
  // vez con v = 1 mm/s. Se elige el radio mas pequeno (curva mas corta) que deja el cuerpo redondo a
  // distancia segura del poste interior y de la esquina exterior de la celda.
  const float half = 0.5f * cell_, hw = 0.5f * p_.wallMm;
  const float need = 0.5f * p_.bodyDiamMm + 8.0f;  // radio del raton + margen
  const float ramp = 12.0f;
  for (float radius = p_.smoothRadiusMm; radius < 2.0f * cell_; radius += 2.0f) {
    const float w = kDeg / radius;
    const float alpha = w * w / (2.0f * ramp);
    float th = 0, rate = 0, x = 0, y = 0, s = 0;
    const float dt = 0.0005f / (w / 90.0f);
    // Trayectoria en ejes de la entrada (x = avance, y = hacia el lado del giro).
    static float px[2000], py[2000];
    int n = 0;
    for (int i = 0; i < 400000 && th < 90.0f; ++i) {
      const float allowed = sqrtf(2.0f * alpha * (90.0f - th));
      const float target = allowed < w ? allowed : w;
      rate = rate < target ? (rate + alpha * dt > target ? target : rate + alpha * dt) : target;
      if (rate < 1e-4f) rate = 1e-4f;
      th += rate * dt;
      x += cosf(th / kDeg) * dt;
      y += sinf(th / kDeg) * dt;
      s += dt;
      if ((i & 7) == 0 && n < 2000) {
        px[n] = x;
        py[n] = y;
        ++n;
      }
    }
    // Simetrica: avance hasta el centro = x final; la celda tiene su centro en (x, 0) en estos ejes.
    // Poste interior en (x - half, half) [lado del giro], esquina exterior en (x + half - hw, -(half - hw)).
    float minPost = 1e9f, minOuter = 1e9f;
    for (int i = 0; i < n; ++i) {
      const float dxp = px[i] - (x - half), dyp = py[i] - half;
      const float dp = sqrtf(dxp * dxp + dyp * dyp) - hw * 1.42f;
      if (dp < minPost) minPost = dp;
      const float dxo = (x + half - hw) - px[i], dyo = py[i] + (half - hw);
      const float dout = dxo < dyo ? dxo : dyo;  // distancia a la pared exterior mas cercana
      if (dout < minOuter) minOuter = dout;
    }
    smoothDf_ = x;
    smoothDl_ = y;
    smoothArc_ = s;
    smoothRadius_ = radius;
    smoothOk_ = minPost >= need && minOuter >= need - 4.0f && x < cell_ - 10.0f;
    if (smoothOk_ || minOuter < need - 4.0f) break;
  }
}

void Navigator::begin(int x, int y, Dir h) {
  cx_ = x;
  cy_ = y;
  h_ = h;
  s_ = io_.tick();
  t_ = s_.t;
  motion_.reset(s_);
  centerOdo_ = 0;
  gridAngle_ = 0;
  lateral_.reset(0, 100.0f, 4e-4f);
  latAge_ = 1e9f;
  lastOdo_ = 0;
  decided_ = false;
  act_ = Act::None;
  arcStarted_ = false;
  blocked_ = false;
  stalled_ = false;
  stallTimer_ = 0;
  cell_ = p_.cellMm;
  cellRef_ = cell_;
  startStraight();
  halfCorr_ = 0.5f * (cell_ - p_.wallMm);
  sideLatOdo_[0] = sideLatOdo_[1] = -1e9f;
  computeSmoothGeometry();
  char line[64];
  snprintf(line, sizeof(line), "CURVA radio=%d avance=%d %s", static_cast<int>(smoothRadius_),
           static_cast<int>(smoothDf_), smoothOk_ ? "ok" : "NO CABE: giros en el sitio");
  io_.report(line, true);
  side_[0] = SideTrack();
  side_[1] = SideTrack();
  maze_.markVisited(cx_, cy_);
}

float Navigator::frontStopReading() const {
  if (p_.frontStopMm >= 0) return static_cast<float>(p_.frontStopMm);
  return 0.5f * cell_ - 0.5f * p_.wallMm - p_.frontAheadMm;
}

void Navigator::setMotorsFromMotion() {
  float l, r;
  motion_.update(s_, l, r);
  io_.setMotors(l, r);
}

void Navigator::step() {
  s_ = io_.tick();
  t_ = s_.t;
  setMotorsFromMotion();
}

void Navigator::tick() {
  s_ = io_.tick();
  t_ = s_.t;
  setMotorsFromMotion();
  const bool straight = !motion_.turning();
  if (straight) {
    while (cellPos() > 0.5f * cell_) enterNextCell();
    senseSides();
    senseFront();
    steer();
    checkStall();
  } else {
    checkStall();
  }
#ifdef RATON_DEBUG
  if (getenv("NAVDBG") && t_ < atof(getenv("NAVDBG")) && t_ > (getenv("NAVFROM") ? atof(getenv("NAVFROM")) : 0))
    printf("%.3f ang=%.2f ideal=%.2f grid=%.2f steer lat=%.1f age=%.2f pos=%.1f v=%.0f/%.0f w=%.0f/%.0f act=%d dec=%d\n", t_,
           motion_.angle(), motion_.idealAngle(), gridAngle_, lateral_.y(), latAge_, cellPos(), motion_.speed(),
           motion_.measuredSpeed(), motion_.angularRate(), motion_.measuredRate(), (int)act_, (int)decided_);
#endif
  report();
}

void Navigator::enterNextCell() {
  centerOdo_ += cell_;
  ++straightCells_;
  cx_ += dx(h_);
  cy_ += dy(h_);
  maze_.markVisited(cx_, cy_);
  // Lo que se decidio para "la celda siguiente" es ahora la actual: si era seguir recto hay que decidir
  // la siguiente; si era parar o girar se ejecuta aqui.
  if (act_ == Act::Forward) {
    decided_ = false;
    act_ = Act::None;
  }
  char line[48];
  snprintf(line, sizeof(line), "CELL x=%d y=%d w=%d", cx_ - p_.startX + 20, cy_ - p_.startY + 20,
           maze_.wallMask(cx_, cy_));
  io_.report(line, true);
}

// ---------------------------------------------------------------------------------------------
// Sensado
// ---------------------------------------------------------------------------------------------

void Navigator::senseSides() {
  // Prediccion lateral con lo avanzado desde el ultimo ciclo; el error de rumbo estimado pasa a la
  // cuadricula (gridAngle_) para que el control vaya realmente recto.
  const float odo = motion_.position();
  lateral_.predict(odo - lastOdo_, headingError() / kDeg);
  lastOdo_ = odo;
  const float bias = clampf(lateral_.takeBias() * kDeg, -0.5f, 0.5f);
  gridAngle_ -= bias;
  gridCorr_ += bias;
  latAge_ += p_.dt;
  const float psiDeg = headingError();
  if (fabsf(psiDeg) > 8.0f) return;
  const float psi = psiDeg / kDeg;
  const float a = p_.diagAngleDeg / kDeg;
  const float cell = cell_;
  const float v = motion_.measuredSpeed();
  const Range* rr[2] = {&s_.left, &s_.right};
  for (int side = 0; side < 2; ++side) {
    const Range& r = *rr[side];
    SideTrack& tr = side_[side];
    if (!r.fresh) continue;
    const float sign = side == 0 ? 1.0f : -1.0f;  // izquierda +, derecha -
    // Distancia sin medida: si justo antes veia la pared muy cerca, es pared (demasiado cerca para medir).
    int mm = r.mm;
    if (mm >= 0) {
      tr.near = mm < 45 ? tr.near + 1 : 0;
      tr.lastValidMm = mm;
      tr.lastValidT = t_;
    } else if (tr.near >= 2 && t_ - tr.lastValidT < 0.06f) {
      mm = tr.lastValidMm;
    }
    const float odoSample = motion_.position() - v * p_.sensorLatencyS;
    const float rayAng = psi + sign * a;
    // Posicion del sensor (eje en el origen, x = avance, y = izquierda) girada psi.
    const float sx = p_.diagAheadMm * cosf(psi) - sign * p_.diagLateralMm * sinf(psi);
    const float sy = p_.diagAheadMm * sinf(psi) + sign * p_.diagLateralMm * cosf(psi);
    const float ray = mm >= 0 ? static_cast<float>(mm) : 1000.0f;
    float hx = sx + ray * cosf(rayAng);
    const float hy = sy + ray * sinf(rayAng);
    const float lateral = sign * hy;  // distancia perpendicular del eje a la pared, hacia ese lado
    // Pasillo ancho (pista de practica): una lectura mas lejos que la pared lateral pero por debajo del
    // umbral suele ser el rayo que cruza un hueco y da en una pared de la celda vecina. No se usa para
    // nada. En el pasillo estrecho de la competencia esa franja es el propio descentrado: se usa.
    if (mm >= 0 && halfCorr_ > 90.0f && lateral > halfCorr_ + 0.5f * p_.wallMm + 25.0f &&
        lateral < p_.sideWallRatio * halfCorr_)
      continue;
    const bool wall = mm >= 0 && lateral < p_.sideWallRatio * halfCorr_;
    if (!wall) {
      // Sin pared: el rayo "cruza" el lateral a la altura del pasillo nominal.
      hx = sx + (halfCorr_ - sign * sy) / tanf(a);
    }
    const float hitOdo = odoSample + hx;
    const float hitRel = hitOdo - centerOdo_;

    // Correccion del avance con el poste: cambio de pared a hueco (o al reves) entre dos muestras.
    if (tr.valid && tr.wall != wall && fabsf(psiDeg) < 4.0f && hitOdo - tr.hitOdo > 0 && hitOdo - tr.hitOdo < 30.0f) {
      const float mid = 0.5f * (hitOdo + tr.hitOdo);
      const float k = floorf((mid - centerOdo_ - 0.5f * cell) / cell + 0.5f);
      const float boundary = centerOdo_ + 0.5f * cell + k * cell;
      const float expected = boundary + (wall ? -0.5f : 0.5f) * p_.wallMm;
      const float e = mid - expected;
      if (fabsf(e) < 25.0f) applyCorrection(e, 0.3f);
    }
    tr.valid = true;
    tr.wall = wall;
    tr.hitOdo = hitOdo;

    // Voto sobre la pared de la celda donde cae el rayo, lejos de los postes.
    const int k = static_cast<int>(floorf((hitRel + 0.5f * cell) / cell));
    const float u = hitRel + 0.5f * cell - k * cell;
    // En pasillos anchos el rayo da mucho mas lejos y el error de angulo pesa mas: mas margen.
    const float margin = 0.5f * p_.wallMm + 14.0f + fmaxf(0.0f, 0.15f * (cell_ - 160.0f));
    if (k >= 0 && k <= 1 && u > margin && u < cell - margin) {
      const int x = cx_ + k * dx(h_), y = cy_ + k * dy(h_);
      const Dir d = side == 0 ? turnLeft(h_) : turnRight(h_);
      if (maze_.inside(x, y)) maze_.observe(x, y, d, wall, 1);
      // Medida lateral (centrado y correccion del rumbo), solo sobre pared y lejos de los postes.
      if (wall && u > margin + 8.0f && u < cell - margin - 8.0f) {
        // Ancho real del pasillo: izquierda + derecha casi no depende del rumbo ni del descentrado.
        // Asi se calibra solo el "~14 cm aproximadamente" del reglamento y el error de angulo de montaje.
        sideLat_[side] = lateral;
        sideLatOdo_[side] = odoSample;
        const int other = 1 - side;
        if (sideLatOdo_[other] > -1e8f && fabsf(odoSample - sideLatOdo_[other]) < 25.0f) {
          const float half = 0.5f * (lateral + sideLat_[other]);
          const float nominal = 0.5f * (p_.cellMm - p_.wallMm);
          if (fabsf(half - nominal) < 25.0f) halfCorr_ += 0.05f * (half - halfCorr_);
        }
        const float y0 = side == 0 ? halfCorr_ - lateral : lateral - halfCorr_;
        if (fabsf(y0) < 45.0f && lateral_.update(y0, 25.0f)) {
          latAge_ = 0;
          ++wallUpdates_;
        }
      }
    }
  }
}

void Navigator::senseFront() {
  Range r = s_.front;
  if (!r.fresh) return;
  if (r.mm >= 0) {
    // Dos lecturas seguidas muy cerca: de verdad hay pared encima (una lectura falsa suelta no cuenta).
    frontNear_ = r.mm < 45 ? frontNear_ + 1 : 0;
    frontLastMm_ = r.mm;
    frontLastT_ = t_;
  } else if (frontNear_ >= 2 && t_ - frontLastT_ < 0.06f) {
    r.mm = frontLastMm_;  // demasiado cerca para medir: sigue habiendo pared
  }
  const float psiDeg = headingError();
  if (fabsf(psiDeg) > 6.0f) return;
  const float cell = cell_;
  const float v = motion_.measuredSpeed();
  const float odoSample = motion_.position() - v * p_.sensorLatencyS;
  const float sensorRel = odoSample - centerOdo_ + p_.frontAheadMm;  // posicion del sensor en la celda
  const float halfWall = 0.5f * p_.wallMm;
  // Distancia del sensor a la pared de cada frontera k (k = 0: salida de la celda actual).
  int hitK = -1;
  if (r.mm >= 0) {
    // Lectura desde el sensor; la pared esta a r.mm + el offset de parada calibrado frente a la geometria.
    const float calib = frontStopReading() - (0.5f * cell - halfWall - p_.frontAheadMm);
    const float face = sensorRel + static_cast<float>(r.mm) - calib;  // cara de la pared, relativa al centro
    const float kf = (face + halfWall - 0.5f * cell) / cell;
    const int k = static_cast<int>(floorf(kf + 0.5f));
    const float residual = face + halfWall - (0.5f * cell + k * cell);
    if (k >= 0 && k <= 2 && fabsf(residual) < 0.3f * cell) {
      hitK = k;
      const int x = cx_ + k * dx(h_), y = cy_ + k * dy(h_);
      const float dist = face - sensorRel;
      if (maze_.inside(x, y)) {
        maze_.observe(x, y, h_, true, k == 0 ? 2 : 1);
#ifdef RATON_DEBUG
        if (getenv("NAVDBG")) printf("%.3f FRONT k=%d r=%d face=%.0f sensorRel=%.0f psi=%.1f\n", t_, k, r.mm, face, sensorRel, psiDeg);
#endif
        // Corregir el avance con una pared confirmada y cercana (el VL53L0X es preciso a corta distancia).
        if (maze_.votes(x, y, h_) >= 3 && dist < 260.0f && fabsf(psiDeg) < 4.0f) {
          if (learnCell_ && !cellLearned_ && straightCells_ + k >= 2) {
            // Error acumulado desde el inicio del tramo recto repartido entre las celdas recorridas: es lo
            // que le falta (o sobra) al paso de celda. Una vez por tramo.
            cellLearned_ = true;
            const float err = (straightCorr_ + residual) / (straightCells_ + k + 0.5f);
            const float before = cell_;
            cell_ = clampf(cell_ + 0.6f * err, 0.92f * cellRef_, 1.08f * cellRef_);
            if (fabsf(cell_ - before) >= 1.0f) {
              char line[40];
              snprintf(line, sizeof(line), "PASO_CELDA %d mm", static_cast<int>(cell_ + 0.5f));
              io_.report(line, false);
            }
          }
          applyCorrection(residual, 0.3f);
        }
        // Pared justo delante que no deberia estar: parar.
        const bool passing = !decided_ || act_ == Act::Forward || !(cx_ == actX_ && cy_ == actY_);
        const float vv = motion_.speed();
        const float stopDist = vv * vv / (2.0f * p_.decel);
        if (k == 0 && passing && !motion_.turning() && dist < frontStopReading() + stopDist + 30.0f) {
          if (++blockedHits_ >= 2) blocked_ = true;
        }
      }
    }
  }
  if (hitK != 0) blockedHits_ = 0;
  // Fronteras que el rayo cruza sin chocar = abiertas (solo las dos mas cercanas, que se ven bien).
  const float reach = r.mm >= 0 ? sensorRel + static_cast<float>(r.mm) : sensorRel + 260.0f;
  // Sin medida (fuera de alcance o perdida) vale medio voto: dos seguidas para dar la pared por abierta.
  if (r.mm < 0 && ++frontMisses_ < 2) return;
  if (r.mm >= 0) frontMisses_ = 0;
  for (int k = 0; k <= 1; ++k) {
    if (hitK >= 0 && k >= hitK) break;
    const float face = 0.5f * cell + k * cell - halfWall;
    if (face - sensorRel < 20.0f) continue;   // ya la tiene encima
    if (face + 40.0f > reach) break;          // no llega a verla
    const int x = cx_ + k * dx(h_), y = cy_ + k * dy(h_);
    if (maze_.inside(x, y)) maze_.observe(x, y, h_, false, 1);
  }
  if (r.mm < 0) frontMisses_ = 0;
}

void Navigator::applyCorrection(float e, float gain) {
  // e > 0: la celda real esta mas adelante de lo que dice la odometria.
  centerOdo_ += gain * e;
  straightCorr_ += gain * e;
}

void Navigator::startStraight() {
  straightCells_ = 0;
  straightCorr_ = 0;
  cellLearned_ = false;
}

void Navigator::steer() {
  const float psiDeg = headingError();
  if (latAge_ < 0.3f && fabsf(motion_.speed()) > 30.0f) {
    const float target = clampf(-p_.kSteer * lateral_.y(), -p_.steerMaxDeg, p_.steerMaxDeg);
    (void)psiDeg;
    motion_.setSteer(target - p_.steerRateGain * motion_.measuredSpeed() * sinf(psiDeg / kDeg));
  } else {
    motion_.setSteer(0);
  }
}

void Navigator::checkStall() {
  // 1) Las ruedas no siguen a la consigna (bloqueadas contra algo): error de seguimiento sostenido.
  const bool wantMove = fabsf(motion_.speed()) > 40.0f || fabsf(motion_.angularRate()) > 120.0f;
  const bool moving = fabsf(motion_.measuredSpeed()) > 12.0f || fabsf(motion_.measuredRate()) > 25.0f;
  const bool lagging = fabsf(motion_.idealPosition() - motion_.position()) > 20.0f ||
                       fabsf(motion_.idealAngle() - motion_.angle()) > 15.0f;
  if ((wantMove && !moving) || lagging) {
    stallTimer_ += p_.dt;
    if (stallTimer_ > p_.stallS) stalled_ = true;
  } else {
    stallTimer_ = 0;
  }
  // 2) Empujando una pared con las ruedas patinando: los encoders avanzan pero el frontal no se acerca.
  if (motion_.turning() || motion_.speed() < 40.0f || s_.front.mm < 0 || s_.front.mm > 250) {
    pushRefOdo_ = -1e9f;
    return;
  }
  if (!s_.front.fresh) return;
  if (pushRefOdo_ < -1e8f) {
    pushRefOdo_ = motion_.position();
    pushRefFront_ = s_.front.mm;
    return;
  }
  const float advanced = motion_.position() - pushRefOdo_;
  if (advanced > 60.0f) {
    if (pushRefFront_ - s_.front.mm < 20) stalled_ = true;
    pushRefOdo_ = motion_.position();
    pushRefFront_ = s_.front.mm;
  }
}

void Navigator::report() {
  if (t_ - lastReport_ < 0.1f) return;
  lastReport_ = t_;
  const float pos = cellPos();
  const float xmm = (cx_ + 0.5f) * cell_ + pos * dx(h_);
  const float ymm = (cy_ + 0.5f) * cell_ + pos * dy(h_);
  char line[120];
  snprintf(line, sizeof(line), "NAV I%d F%d D%d v%d r%d p%d x%d y%d a%d", s_.left.mm, s_.front.mm, s_.right.mm,
           static_cast<int>(motion_.measuredSpeed()), static_cast<int>(wrap180(motion_.angle())),
           static_cast<int>(cell_ - p_.wallMm), static_cast<int>(xmm), static_cast<int>(ymm),
           static_cast<int>(wrap180(headingError())));
  io_.report(line, false);
}

// ---------------------------------------------------------------------------------------------
// Movimiento
// ---------------------------------------------------------------------------------------------

Navigator::Act Navigator::actionFor(int dir) const {
  if (dir < 0) return Act::Stop;
  const Dir d = static_cast<Dir>(dir);
  if (d == h_) return Act::Forward;
  if (d == turnLeft(h_)) return Act::Left;
  if (d == turnRight(h_)) return Act::Right;
  return Act::Back;
}

void Navigator::plan(Brain& brain) {
  const int nx = cx_ + dx(h_), ny = cy_ + dy(h_);
  decided_ = true;
  if (!maze_.passable(cx_, cy_, h_, true) || !maze_.inside(nx, ny)) {
    // Delante hay pared: parar en el centro de esta celda y decidir alli.
    blocked_ = true;
    return;
  }
  const int dir = brain.decide(nx, ny, h_);
  act_ = actionFor(dir);
  nextDir_ = dir;
  actX_ = nx;
  actY_ = ny;
  arcPlanned_ = false;
  // Punto final del tramo: celda (planX_, planY_) y si se llega a ella en curva o parando. El objetivo
  // de avance se recalcula en cada ciclo desde la posicion corregida (postes, pared de frente).
  planActive_ = true;
  planArc_ = false;
  planX_ = nx;
  planY_ = ny;
  const float cCenter = centerOdo_ + cell_;
  if (act_ == Act::Forward) {
    const int k = brain.straightAfter(nx, ny, h_);
    if (k >= 0) {
      planX_ = nx + (k + 1) * dx(h_);
      planY_ = ny + (k + 1) * dy(h_);
      planArc_ = smooth_ && smoothOk_;
      planVEnd_ = planArc_ ? p_.turnSpeed : 0.0f;
    } else {
      planActive_ = false;
      motion_.cruise(vmax_);
    }
  } else if ((act_ == Act::Left || act_ == Act::Right) && smooth_ && smoothOk_ &&
             motion_.position() < cCenter - smoothDf_ - 5.0f) {
    planArc_ = true;
    planVEnd_ = p_.turnSpeed;
    arcPlanned_ = true;
  } else {
    planVEnd_ = 0;
  }
  if (planActive_) motion_.moveTo(planTarget(), vmax_, planVEnd_);
  arcStarted_ = false;
}

bool Navigator::waitStopped(float timeout) {
  const float t0 = t_;
  while (t_ - t0 < timeout) {
    tick();
    if (io_.stopRequested()) return false;
    if (motion_.arrived() && !motion_.turning() && fabsf(motion_.measuredSpeed()) < 15.0f &&
        fabsf(motion_.measuredRate()) < 30.0f)
      return true;
  }
  return false;
}

bool Navigator::pivot(int q) {
  if (q == 0) return true;
  // Media vuelta: hacia el lado con mas sitio (el raton es redondo, da igual salvo por el roce).
  if (q == 2 || q == -2) q = (s_.right.mm >= 0 && (s_.left.mm < 0 || s_.right.mm > s_.left.mm)) ? -2 : 2;
  learnGyroScale();
  turnSince_ += 90.0f * q;
  const float target = gridAngle_ + 90.0f * q;
  motion_.turnTo(target, p_.rotSpeed, p_.rotAccel);
  const float t0 = t_;
  stalled_ = false;
  while (t_ - t0 < 2.0f) {
    tick();
    if (io_.stopRequested()) return false;
    if (stalled_) break;
    if (!motion_.turning() && fabsf(motion_.angle() - target) < 3.0f && fabsf(motion_.measuredRate()) < 40.0f)
      break;
  }
  gridAngle_ = target;
  h_ = static_cast<Dir>((h_ + 4 - q) & 3);  // q > 0 = izquierda (antihorario)
  centerOdo_ = motion_.position();
  startStraight();
  if (q == 2 || q == -2) {
    lateral_.setY(-lateral_.y(), lateral_.varY() + 4.0f);
  } else {
    lateral_.setY(0, 100.0f);  // el avance de antes es ahora el lateral: ~centrado por el frontal
  }
  lateral_.addHeadingVariance(1.5e-3f);  // ~2 grados por el error de escala del giroscopio
  lastOdo_ = motion_.position();
  latAge_ = 1e9f;
  side_[0] = SideTrack();
  side_[1] = SideTrack();
  blockedHits_ = 0;
  frontNear_ = 0;
  frontLastT_ = -1;
  return true;
}

void Navigator::learnGyroScale() {
  // Tras girar turnSince_ grados, el filtro lateral ha tenido que corregir gridCorr_ grados con las paredes:
  // si es proporcional al giro, el giroscopio mide de menos (o de mas). Se ajusta la escala poco a poco.
  if (p_.learnGyro && fabsf(turnSince_) >= 89.0f && wallUpdates_ >= 12) {
    const float eps = clampf(gridCorr_ / turnSince_, -0.05f, 0.05f);
    const float k = clampf(motion_.gyroScale() * (1.0f + 0.3f * eps), 0.93f, 1.07f);
    motion_.setGyroScale(k);
  }
  if (wallUpdates_ >= 12 || fabsf(turnSince_) > 400.0f) {
    turnSince_ = 0;
    gridCorr_ = 0;
  }
  wallUpdates_ = 0;
}

void Navigator::face(Dir d) {
  const int q = ((h_ - d) + 4) & 3;  // vueltas a la izquierda
  pivot(q == 3 ? -1 : q);
  motion_.hold(s_);
  waitStopped(0.3f);
}

bool Navigator::wallAheadOpenConfirmed() const { return maze_.votes(cx_, cy_, h_) <= -3; }

bool Navigator::checkFrontAfterPivot() {
  if (wallAheadOpenConfirmed()) return true;
  // La primera medida nueva pudo empezar girando: se usa la segunda.
  int fresh = 0;
  const float t0 = t_;
  int mm = -1;
  while (fresh < 2 && t_ - t0 < 0.08f) {
    tick();
    if (s_.front.fresh) {
      ++fresh;
      mm = s_.front.mm;
    }
  }
  if (fresh < 2) return maze_.passable(cx_, cy_, h_, true);
  const bool wall = mm >= 0 && mm < frontStopReading() + 0.45f * cell_;
  maze_.observe(cx_, cy_, h_, wall, 2);
  return !wall;
}

Navigator::Depart Navigator::depart(Brain& brain, float vmax) {
  // Quieto en el centro de (cx, cy): decide, gira en el sitio y comprueba con el frontal.
  for (int tries = 0; tries < 6; ++tries) {
    if (io_.stopRequested() || brain.shouldAbort()) return Depart::Fail;
    const int dir = brain.decide(cx_, cy_, h_);
    if (dir < 0) return Depart::Stop;
    const Act a = actionFor(dir);
    const int q = a == Act::Left ? 1 : (a == Act::Right ? -1 : (a == Act::Back ? 2 : 0));
    if (!pivot(q)) return Depart::Fail;
    if (checkFrontAfterPivot()) {
      decided_ = false;
      act_ = Act::None;
      blocked_ = false;
      stalled_ = false;
      planActive_ = false;
      arcPlanned_ = false;
      motion_.cruise(vmax);
      return Depart::Go;
    }
  }
  char line[72];
  snprintf(line, sizeof(line), "NO_SALE x=%d y=%d: el frontal ve pared en todas las salidas (F%d)", cx_, cy_,
           s_.front.mm);
  io_.report(line, true);
  return Depart::Fail;
}

void Navigator::centerOnFront(int k) {
  // Con una pared delante (frontera k), mover el eje al centro de la celda usando el frontal.
  motion_.hold(s_);
  int l, f, r;
  sample(2, l, f, r);
  if (f >= 0) {
    const float face = 0.5f * cell_ - 0.5f * p_.wallMm + k * cell_;
    const float calib = frontStopReading() - (0.5f * cell_ - 0.5f * p_.wallMm - p_.frontAheadMm);
    const float pos = face - (f - calib) - p_.frontAheadMm;  // eje respecto al centro
    if (fabsf(pos) < 60.0f) {
      motion_.moveTo(motion_.position() - pos, 150.0f, 0);
      waitStopped(0.8f);
    }
  }
  centerOdo_ = motion_.position();
  startStraight();
  decided_ = false;
  act_ = Act::None;
}

bool Navigator::recover(Brain& brain) {
  (void)brain;
  ++stalls_;
  ++recoveries_;
  char line[64];
  snprintf(line, sizeof(line), "ATASCO x=%d y=%d", cx_, cy_);
  io_.report(line, true);
  const int idx = maze_.index(cx_, cy_);
  recoverCount_ = idx == recoverCellIdx_ ? recoverCount_ + 1 : 1;
  recoverCellIdx_ = idx;
  // 1) Parar y retroceder en linea recta.
  motion_.hold(s_);
  stalled_ = false;
  stallTimer_ = 0;
  pushRefOdo_ = -1e9f;
  const float back = motion_.position() - p_.backoffMm;
  motion_.moveTo(back, 150.0f, 0);
  waitStopped(1.2f);
  // 2) Enderezarse a la cuadricula.
  motion_.turnTo(gridAngle_, p_.rotSpeed * 0.5f, p_.rotAccel);
  waitStopped(1.0f);
  stalled_ = false;
  // 3) Recentrar en el pasillo si los laterales ven que esta desplazado (si no, chocaria otra vez con el
  //    mismo poste): paso en "S" de 35 grados.
  int l, f, r;
  sample(3, l, f, r);
  {
    const float sa = sinf(p_.diagAngleDeg / kDeg);
    const float latL = l >= 0 ? p_.diagLateralMm + l * sa : 1e9f;
    const float latR = r >= 0 ? p_.diagLateralMm + r * sa : 1e9f;
    const float wallMax = p_.sideWallRatio * halfCorr_;
    float y = 0;
    bool known = true;
    if (latL < wallMax && latR < wallMax) y = 0.5f * (latR - latL);
    else if (latL < wallMax) y = halfCorr_ - latL;
    else if (latR < wallMax) y = latR - halfCorr_;
    else known = false;
    if (known && fabsf(y) > 10.0f && fabsf(y) < 50.0f) {
      const float th = 35.0f;
      const float sgn = y > 0 ? -1.0f : 1.0f;  // desplazado a la izquierda: ir a la derecha
      const float d = fabsf(y) / sinf(th / kDeg);
      motion_.turnTo(gridAngle_ + sgn * th, p_.rotSpeed * 0.5f, p_.rotAccel);
      waitStopped(0.8f);
      motion_.moveTo(motion_.position() + d, 150.0f, 0);
      waitStopped(1.0f);
      motion_.turnTo(gridAngle_, p_.rotSpeed * 0.5f, p_.rotAccel);
      waitStopped(0.8f);
      motion_.moveTo(motion_.position() - d * cosf(th / kDeg), 150.0f, 0);
      waitStopped(1.0f);
      lateral_.setY(0, 25.0f);
      stalled_ = false;
      sample(2, l, f, r);
    }
  }
  // 4) Si el frontal ve una pared cerca, es la de esta celda: ubicar el avance con ella.
  if (f >= 0 && f < 0.5f * cell_ + 40.0f) {
    const float face = 0.5f * cell_ - 0.5f * p_.wallMm;
    const float calib = frontStopReading() - (face - p_.frontAheadMm);
    const float pos = face - (f - calib) - p_.frontAheadMm;
    centerOdo_ = motion_.position() - pos;
    startStraight();
    maze_.observe(cx_, cy_, h_, true, 3);
  } else if (recoverCount_ >= 2) {
    // Varios atascos aqui sin pared visible: tratar la salida de frente como cerrada.
    maze_.observe(cx_, cy_, h_, true, 3);
  }
  // 5) Al centro de la celda.
  motion_.moveTo(centerOdo_, 150.0f, 0);
  waitStopped(1.2f);
  stalled_ = false;
  decided_ = false;
  act_ = Act::None;
  blocked_ = false;
  return true;
}

void Navigator::sample(int n, int& left, int& front, int& right) {
  int cnt[3] = {0, 0, 0};
  long sum[3] = {0, 0, 0};
  int got[3] = {0, 0, 0};
  const float t0 = t_;
  while ((got[0] < n || got[1] < n || got[2] < n) && t_ - t0 < 0.03f * n + 0.1f) {
    step();
    const Range* rr[3] = {&s_.left, &s_.front, &s_.right};
    for (int i = 0; i < 3; ++i) {
      if (!rr[i]->fresh) continue;
      ++got[i];
      if (rr[i]->mm >= 0) {
        sum[i] += rr[i]->mm;
        ++cnt[i];
      }
    }
  }
  left = cnt[0] ? static_cast<int>(sum[0] / cnt[0]) : -1;
  front = cnt[1] ? static_cast<int>(sum[1] / cnt[1]) : -1;
  right = cnt[2] ? static_cast<int>(sum[2] / cnt[2]) : -1;
}

float Navigator::planTarget() const {
  const int n = (planX_ - cx_) * dx(h_) + (planY_ - cy_) * dy(h_);
  return centerOdo_ + n * cell_ - (planArc_ ? smoothDf_ : 0.0f);
}

float Navigator::actCenterOdo() const {
  return (cx_ == actX_ && cy_ == actY_) ? centerOdo_ : centerOdo_ + cell_;
}

NavResult Navigator::run(Brain& brain, float vmax, bool smoothTurns) {
  vmax_ = vmax;
  smooth_ = smoothTurns;
  stalled_ = false;
  blocked_ = false;
  decided_ = false;
  act_ = Act::None;
  // Arranca quieto en el centro de una celda.
  Depart dep = depart(brain, vmax);
  while (dep == Depart::Go) {
    tick();
    if (io_.stopRequested() || brain.shouldAbort()) {
      dep = Depart::Fail;
      break;
    }
    if (stalled_) {
      recover(brain);
      // Tras un atasco el resto del recorrido va en modo seguro: giros en el sitio y velocidad de
      // exploracion. Mas vale llegar que arriesgar el intento.
      if (smooth_ || vmax_ > p_.searchSpeed) {
        smooth_ = false;
        vmax_ = vmax = p_.searchSpeed;
        io_.report("MODO_SEGURO", true);
      }
      dep = depart(brain, vmax);
      continue;
    }
    if (blocked_) {
      // Pared delante: parar sin tocarla y quedar en el centro de esta celda.
      motion_.moveTo(centerOdo_ > motion_.position() ? centerOdo_ : motion_.position(), vmax_, 0);
      waitStopped(1.0f);
      centerOnFront(0);
      blocked_ = false;
      dep = depart(brain, vmax);
      continue;
    }
    // Explorando se decide al ver las paredes de la celda siguiente; en carrera (ruta conocida) antes,
    // para que la curva pueda empezar en la celda anterior.
    const float trigger = smooth_ ? 0.0f : 0.5f * cell_ - 22.0f;
    if (!decided_ && cellPos() >= trigger) plan(brain);
    if (!decided_) continue;
    if (planActive_ && !arcStarted_) {
      const float target = planTarget();
      if (!motion_.arrived() && fabsf(target - motion_.endPosition()) > 0.5f) motion_.moveTo(target, vmax_, planVEnd_);
    }
    if (act_ == Act::Forward || act_ == Act::None) continue;

    const bool turnAct = act_ == Act::Left || act_ == Act::Right;
    if (turnAct && arcPlanned_ && !arcStarted_ && motion_.position() >= actCenterOdo() - smoothDf_ - 0.5f) {
      // Curva suave: girar a velocidad constante desde aqui (puede empezar aun en la celda anterior).
      while (!(cx_ == actX_ && cy_ == actY_) && maze_.inside(cx_ + dx(h_), cy_ + dy(h_))) enterNextCell();
      const float v = motion_.speed() > 80.0f ? motion_.speed() : 80.0f;
      const float w = v / smoothRadius_ * kDeg;
      const float alpha = w * w / (2.0f * 12.0f);
      const int q = act_ == Act::Left ? 1 : -1;
      motion_.moveTo(1e9f, v, v);
      motion_.turnTo(gridAngle_ + 90.0f * q, w, alpha);
      arcStarted_ = true;
      planActive_ = false;
      continue;
    }
    if (arcStarted_) {
      if (motion_.turning()) continue;
      const int q = act_ == Act::Left ? 1 : -1;
      gridAngle_ += 90.0f * q;
      h_ = q > 0 ? turnLeft(h_) : turnRight(h_);
      // Fin de la curva: el eje queda smoothDl_ pasado el centro de la celda, ya en el nuevo rumbo.
      centerOdo_ = motion_.position() - smoothDl_;
      startStraight();
      lateral_.setY(0, 100.0f);
      lateral_.addHeadingVariance(7e-4f);
      lastOdo_ = motion_.position();
      latAge_ = 1e9f;
      side_[0] = SideTrack();
      side_[1] = SideTrack();
      frontNear_ = 0;
      frontLastT_ = -1;
      arcStarted_ = false;
      arcPlanned_ = false;
      decided_ = false;
      act_ = Act::None;
      motion_.cruise(vmax_);
      continue;
    }
    if (arcPlanned_) continue;
    // Parada en el centro de la celda de la accion (el objetivo ya se corrige arriba).
    if (motion_.arrived() && cx_ == actX_ && cy_ == actY_) {
      waitStopped(0.4f);
      centerOdo_ = motion_.position();
      if (act_ == Act::Stop) {
        dep = Depart::Stop;
        break;
      }
      decided_ = false;
      act_ = Act::None;
      dep = depart(brain, vmax);
    }
  }
  motion_.hold(s_);
  io_.setMotors(0, 0);
  decided_ = false;
  act_ = Act::None;
  if (dep == Depart::Stop) return NavResult::Arrived;
  if (io_.stopRequested() || brain.shouldAbort()) return NavResult::Aborted;
  return NavResult::NoPath;
}

}  // namespace raton
