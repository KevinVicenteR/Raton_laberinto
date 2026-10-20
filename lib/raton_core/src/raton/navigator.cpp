#include "raton/navigator.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#ifdef RATON_DEBUG
#include <chrono>
#endif

namespace raton {

#ifdef RATON_DEBUG
double gRadarSeconds = 0, gRadarMax = 0;  // tiempo del radar en el PC (para estimar la placa)
long gRadarCalls = 0;
#endif

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
  // vez con v = 1 mm/s. De los radios que dejan el cuerpo redondo a distancia segura del poste interior y
  // de la esquina exterior, el mas rapido: con un radio mayor la rueda de fuera va menos por encima del
  // centro y la curva se puede tomar mas deprisa con los mismos motores. Que empiece y acabe dentro de media
  // celda, para poder enlazar dos curvas seguidas sin parar.
  const float half = 0.5f * cell_, hw = 0.5f * p_.wallMm;
  const float need = 0.5f * p_.bodyDiamMm + 8.0f;  // radio del raton + margen
  const float ramp = 12.0f;
  float bestT = 1e9f, bestR = -1, bestX = 0, bestY = 0, bestS = 0;
  float firstR = -1, firstX = 0, firstY = 0, firstS = 0;  // la mas cerrada que cabe (como antes)
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
    if (minOuter < need - 4.0f) break;  // mas abierta ya no cabe
    const bool fits = minPost >= need && x < cell_ - 10.0f;
    if (!fits) continue;
    if (firstR < 0) {
      firstR = radius;
      firstX = x;
      firstY = y;
      firstS = s;
    }
    if (x > half - 3.0f) continue;
    const float t = (s + 2.0f * (half - x)) / arcSpeed(radius);  // de media celda antes a media despues
    if (t < bestT) {
      bestT = t;
      bestR = radius;
      bestX = x;
      bestY = y;
      bestS = s;
    }
  }
  if (bestR < 0) {
    bestR = firstR;
    bestX = firstX;
    bestY = firstY;
    bestS = firstS;
  }
  smoothOk_ = bestR > 0;
  smoothRadius_ = smoothOk_ ? bestR : p_.smoothRadiusMm;
  smoothDf_ = bestX;
  smoothDl_ = bestY;
  smoothArc_ = bestS;
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
  // El paso de celda aprendido se conserva de una mision a otra (explorar y luego la carrera) salvo que
  // cambie el parametro.
  if (p_.cellMm != cellRef_) {
    cell_ = cellRef_ = p_.cellMm;
    cellUpdates_ = 0;
  }
  startStraight();
  halfCorr_ = 0.5f * (cell_ - p_.wallMm);
  sonda_.setup(cell_, p_.wallMm, p_.sensorFovDeg, p_.sensorMaxMm);
  // Lo vertical se aprende en marcha: se conserva de una mision a otra salvo que cambien los parametros.
  const float key[7] = {p_.sensorHeightMm, p_.sensorTiltDeg, p_.wallHeightMm, p_.floorReflect,
                        p_.leftTiltDeg,    p_.frontTiltDeg,  p_.rightTiltDeg};
  bool same = true;
  for (int i = 0; i < 7; ++i) same = same && key[i] == vertKey_[i];
  if (!same) {
    sonda_.setVertical(p_.sensorHeightMm, p_.sensorTiltDeg, p_.wallHeightMm, p_.floorReflect);
    sonda_.setTilt(0, p_.sensorTiltDeg + p_.leftTiltDeg);
    sonda_.setTilt(1, p_.sensorTiltDeg + p_.frontTiltDeg);
    sonda_.setTilt(2, p_.sensorTiltDeg + p_.rightTiltDeg);
    for (int i = 0; i < 7; ++i) vertKey_[i] = key[i];
  }
  longVar_ = 25.0f;
  computeSmoothGeometry();
  char line[64];
  snprintf(line, sizeof(line), "CURVA radio=%d avance=%d %s", static_cast<int>(smoothRadius_),
           static_cast<int>(smoothDf_), smoothOk_ ? "ok" : "NO CABE: giros en el sitio");
  io_.report(line, true);
  resetBeams();
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

float Navigator::rangeBias() const {
  // Los desfases se calibraron suponiendo pasillo = celda - pared: si el paso real (aprendido con la odometria
  // en las rectas) es otro, las tres lecturas quedan cortas o largas la mitad de la diferencia.
  const float ref = offsetsCell_ > 0 ? offsetsCell_ : cellRef_;
  return clampf(0.5f * (cell_ - ref), -10.0f, 10.0f);
}

void Navigator::tick() {
  s_ = io_.tick();
  t_ = s_.t;
  const int bias = p_.rangeBiasFromCell ? static_cast<int>(floorf(rangeBias() + 0.5f)) : 0;
  if (bias != 0) {
    Range* rr[3] = {&s_.left, &s_.front, &s_.right};
    for (int i = 0; i < 3; ++i)
      if (rr[i]->mm >= 0) rr[i]->mm += bias;
  }
  setMotorsFromMotion();
  const bool straight = !motion_.turning();
  if (straight) {
    while (cellPos() > 0.5f * cell_) enterNextCell();
    sense();
    steer();
    checkStall();
  } else {
    // Girando: lo que se mueva el eje no es avance en el rumbo de la celda (en un giro en el sitio las ruedas
    // patinan algo). Sin esto, al acabar el giro se sumaria al desplazamiento lateral con el rumbo nuevo.
    lastOdo_ = motion_.position();
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

void Navigator::resetBeams() {
  for (int i = 0; i < 3; ++i) beam_[i] = Beam();
  blockedHits_ = 0;
}

void Navigator::sensorPose(int sensor, float fwd, float lat, float dpsi, float& x, float& y, float& ang) const {
  // Pose del eje en el instante de la medida (la lectura llega sensorLatencyS tarde).
  const float pos = cellPos() - motion_.measuredSpeed() * p_.sensorLatencyS + fwd;
  const float yl = lateral_.y() + lat;
  const float fx = static_cast<float>(dx(h_)), fy = static_cast<float>(dy(h_));
  const float ax = (cx_ + 0.5f) * cell_ + fx * pos - fy * yl;
  const float ay = (cy_ + 0.5f) * cell_ + fy * pos + fx * yl;
  const float th = atan2f(fy, fx) + headingError() / kDeg + dpsi;
  const float c = cosf(th), sn = sinf(th);
  float ahead = p_.frontAheadMm, side = 0, mount = 0;
  if (sensor != 1) {
    const float sign = sensor == 0 ? 1.0f : -1.0f;
    ahead = p_.diagAheadMm;
    side = sign * p_.diagLateralMm;
    mount = sign * p_.diagMountDeg / kDeg;
  }
  x = ax + ahead * c - side * sn;
  y = ay + ahead * sn + side * c;
  ang = th + mount;
}

void Navigator::sense() {
  // Prediccion lateral con lo avanzado; el error de rumbo estimado pasa a la cuadricula (gridAngle_) para
  // que el control vaya realmente recto.
  const float odo = motion_.position();
  const float ds = odo - lastOdo_;
  // Con el rumbo lejos de la cuadricula (acabando un giro, antes de cambiar de celda) el modelo lineal del
  // desplazamiento no vale.
  if (fabsf(headingError()) < 20.0f) lateral_.predict(ds, headingError() / kDeg);
  longVar_ += (learnCell_ ? 0.3f : 0.2f) * fabsf(ds);
  lastOdo_ = odo;
  const float bias = clampf(lateral_.takeBias() * kDeg, -0.5f, 0.5f);
  gridAngle_ -= bias;
  gridCorr_ += bias;
  latAge_ += p_.dt;
  const Range* rr[3] = {&s_.left, &s_.front, &s_.right};
  for (int i = 0; i < 3; ++i) {
    const Range& r = *rr[i];
    if (!r.fresh) continue;
    Beam& b = beam_[i];
    int mm = r.mm;
    if (mm >= 0) {
      b.near = mm < 45 ? b.near + 1 : 0;
      b.lastValidMm = mm;
      b.lastValidT = t_;
      b.misses = 0;
    } else {
      ++b.misses;
      // Dos lecturas seguidas muy cerca y ahora ninguna: demasiado cerca para medir, sigue habiendo pared.
      if (b.near >= 2 && t_ - b.lastValidT < 0.06f) mm = b.lastValidMm;
    }
    if (i == 1) frontCheck(mm);
    // Girando (o acabando de girar) la lectura es de otro rumbo: con 12 ms de retraso, a 100 grados/s ya
    // son mas de 1 grado.
    if (fabsf(headingError()) > 8.0f || fabsf(motion_.measuredRate()) > 40.0f) continue;
    // Una lectura perdida suelta no dice nada (pasa a veces); dos seguidas = no hay nada a su alcance.
    if (mm < 0 && b.misses < 2) continue;
#ifdef RATON_DEBUG
    const auto t0 = std::chrono::steady_clock::now();
    radar(i, mm);
    const double dtr = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    gRadarSeconds += dtr;
    if (dtr > gRadarMax) gRadarMax = dtr;
    ++gRadarCalls;
#else
    radar(i, mm);
#endif
  }
}

bool Navigator::confident(const Sonda::Scan& scan, float reach) const {
  for (int j = 0; j < Sonda::kRays; ++j) {
    const Sonda::Ray& r = scan.rays[j];
    for (int k = 0; k < r.n; ++k) {
      const Sonda::Hit& h = r.hits[k];
      if (h.t > reach || h.seg == Sonda::kSolid) break;
      const int v = maze_.segVotes(h.seg);
      if (v < 3 && v > -3) return false;
      if (v > 0) break;
    }
  }
  return true;
}

void Navigator::radar(int sensor, int mm) {
  float x, y, ang;
  sensorPose(sensor, 0, 0, 0, x, y, ang);
  sonda_.trace(maze_, x, y, ang, scan_, sensor);
  const float far = static_cast<float>(sonda_.maxMm()) + 100.0f;
  auto val = [far](float r) { return r < 0 ? far : r; };
  const float m = val(static_cast<float>(mm));

  // Sensibilidad de la lectura a la pose (con el mapa de ahora): cuanto cambia al mover el raton 2 mm
  // adelante o a la izquierda. Sirve para la tolerancia del mapa y para corregir la pose.
  float floor0 = 0;  // duda por el piso de la prediccion con el mapa de ahora
  const float h0 = sonda_.predict(maze_, scan_, nullptr, floor0);
  float hx = 0, hy = 0;
  bool slopes = h0 >= 0;
  if (slopes) {
    for (int q = 0; q < 2; ++q) {
      sensorPose(sensor, q == 0 ? 2.0f : 0, q == 1 ? 2.0f : 0, 0, x, y, ang);
      sonda_.trace(maze_, x, y, ang, scanJ_, sensor);
      const float hq = sonda_.predict(maze_, scanJ_);
      if (hq < 0) {
        slopes = false;
        break;
      }
      (q == 0 ? hx : hy) = (hq - h0) * 0.5f;
    }
  }
  // Tolerancia: ruido del sensor, error del modelo y lo que la pose dudosa mueve esta lectura. Junto a un
  // salto (borde de un poste) la pendiente es enorme y la tolerancia tambien: casi no vota.
  const float hxc = slopes ? clampf(fabsf(hx), 0, 6.0f) : 1.0f, hyc = slopes ? clampf(fabsf(hy), 0, 6.0f) : 1.0f;
  const float range = static_cast<float>(mm >= 0 ? mm : sonda_.maxMm());
  const float noise = 3.0f + 0.035f * range;
  // Error del modelo: crece con la distancia (angulo de montaje, desfase del sensor, pista de otra medida).
  const float model2 = 9.0f + 0.0025f * range * range;
  const float tol = 1.3f * sqrtf(noise * noise + model2 + hxc * hxc * longVar_ + hyc * hyc * lateral_.varY());

  // ---- Mapa: paredes que el cono alcanza, de la mas cercana a la mas lejana ----
  int cand[8];
  float candT[8];
  int nc = 0;
  for (int j = 0; j < Sonda::kRays; ++j) {
    const Sonda::Ray& r = scan_.rays[j];
    for (int k = 0; k < r.n; ++k) {
      const Sonda::Hit& h = r.hits[k];
      if (h.seg == Sonda::kSolid) break;
      int q = 0;
      while (q < nc && cand[q] != h.seg) ++q;
      if (q < nc) {
        if (h.t < candT[q]) candT[q] = h.t;
      } else if (nc < 8) {
        cand[nc] = h.seg;
        candT[nc] = h.t;
        ++nc;
      }
    }
  }
  for (int a = 1; a < nc; ++a)
    for (int b = a; b > 0 && candT[b] < candT[b - 1]; --b) {
      const int ts = cand[b];
      cand[b] = cand[b - 1];
      cand[b - 1] = ts;
      const float tt = candT[b];
      candT[b] = candT[b - 1];
      candT[b - 1] = tt;
    }
  // Todas las combinaciones de las paredes mas cercanas (hasta 5): las que explican la lectura dicen que
  // se sabe. Una pared se vota solo si todas las explicaciones buenas coinciden en ella; si las hay con
  // pared y sin pared, esta lectura no dice nada de ella. Las mas lejanas quedan como estan en el mapa.
  // Solo paredes que el sensor ha podido ver y no muy lejos (de lejos el cono es una mancha grande y la
  // lectura poco fiable).
  // Laterales: las paredes de esta celda y la siguiente; frontal: hasta dos celdas por delante.
  const float reachMax = 0.75f * static_cast<float>(sonda_.maxMm());
  const float voteReach = clampf(sensor == 1 ? 2.2f * cell_ : cell_ + 40.0f, 0, reachMax);
  // Se prueban todas las que el sensor ha podido ver (las lejanas tambien explican la lectura), pero solo se
  // votan las cercanas.
  while (nc > 0 && candT[nc - 1] > m + tol) --nc;
  if (nc > 6) nc = 6;
  sonda_.prepare(maze_, scan_, cand, nc, combo_);
  int allClosed = (1 << nc) - 1, allOpen = (1 << nc) - 1;  // bits que valen 1 / 0 en todas las buenas
  int good = 0;
  const float goodTol = 1.5f * tol;
  for (int mask = 0; mask < (1 << nc); ++mask) {
    float fsig;
    const float pr = sonda_.predictMask(combo_, mask, &fsig);
    // Sin medida = la media del cono pasa del alcance: cualquier prediccion lejana (> 60 % del alcance) lo
    // explica, el modelo de lejos no es fino. Solo descarta paredes que darian un eco claro y cercano.
    // Si buena parte de lo que vuelve es piso, la prediccion es dudosa: mas margen (con el sensor bajo, el
    // piso a una celda se parece a una pared).
    const bool ok = mm >= 0 ? (pr >= 0 && fabsf(pr - m) < sqrtf(goodTol * goodTol + 4.0f * fsig * fsig))
                            : (pr < 0 || pr > 0.6f * static_cast<float>(sonda_.maxMm()));
    if (!ok) continue;
    ++good;
    allClosed &= mask;
    allOpen &= ~mask;
  }
  // Ninguna explica la lectura: la pose esta mal, no se vota (abajo puede corregirse con lo confirmado).
  if (good > 0 && good < (1 << nc)) {
    for (int k = 0; k < nc; ++k) {
      if (candT[k] > voteReach) break;
#ifdef RATON_DEBUG
      if (getenv("VOTEDBG") && atoi(getenv("VOTEDBG")) == cand[k])
        printf("%.3f VOTO seg %d s%d mm=%d tol=%.1f buenas=%d/%d pared=%d hueco=%d t=%.0f h0=%.1f\n", t_, cand[k], sensor,
               mm, tol, good, 1 << nc, (allClosed >> k) & 1, (allOpen >> k) & 1, candT[k], h0);
#endif
      // Sin medida no se deduce ninguna pared cerrada (solo que lo cercano esta libre).
      if (((allClosed >> k) & 1) && mm >= 0) maze_.segObserve(cand[k], true, 1);
      else if ((allOpen >> k) & 1) maze_.segObserve(cand[k], false, 1);
    }
  }

  learnVertical(sensor, mm, h0);

  // ---- Pose: solo con lo que ya esta confirmado y cerca (de lejos el cono roza paredes en angulo y un
  //      pequeno error del modelo moveria mucho la pose) ----
  const float nearMax = sensor == 1 ? 1.5f * cell_ : 0.9f * cell_ + 40.0f;
  if (mm < 0 || mm > nearMax || !slopes || !confident(scan_, m + tol)) return;
  // Un salto en la prediccion (el cono justo en el borde de un poste) no es una pendiente: no usar.
  if (fabsf(hx) > 4.0f || fabsf(hy) > 4.0f) return;
  float inn = static_cast<float>(mm) - h0;
  if (fabsf(inn) > 3.0f * tol + 10.0f) return;
  const float r = noise * noise + model2 + floor0 * floor0;
  if (fabsf(hy) > 0.15f) {
    // El rumbo NO se corrige con la forma de una sola lectura: un desfase de unos mm de un sensor (o su
    // angulo de montaje) lo torceria grados. El rumbo sale de como cambia el desplazamiento al avanzar.
    float dyc, dbc;
    const bool okU = lateral_.updateLinear(inn, hy, 0.0f, r, dyc, dbc);
#ifdef RATON_DEBUG
    if (getenv("RADARDBG") && t_ > atof(getenv("RADARDBG")) && t_ < atof(getenv("RADARDBG")) + 1.5)
      printf("%.3f LAT s%d mm=%d h0=%.1f inn=%.1f hx=%.2f hy=%.2f ok=%d dy=%.2f y=%.1f varY=%.1f\n", t_, sensor, mm, h0,
             inn, hx, hy, okU, dyc, lateral_.y(), lateral_.varY());
#endif
    if (!okU) return;
    latAge_ = 0;
    ++wallUpdates_;
    inn -= hy * dyc;
  }
  if (fabsf(hx) > 0.2f) {
    const float sv = hx * hx * longVar_ + r;
    if (inn * inn > 9.0f * sv) return;
    // Cerca del borde de una pared la lectura no es lineal: comprobar con la sonda que el desplazamiento
    // que pide la lectura de verdad la explica antes de creerla.
    const float full = clampf(inn / hx, -40.0f, 40.0f);
    if (fabsf(inn) > 2.0f * sqrtf(r)) {
      sensorPose(sensor, full, 0, 0, x, y, ang);
      sonda_.trace(maze_, x, y, ang, scanJ_, sensor);
      const float hv = sonda_.predict(maze_, scanJ_);
      if (hv < 0 || fabsf(static_cast<float>(mm) - hv) > 0.5f * fabsf(inn)) return;
    }
    const float kx = longVar_ * hx / sv;
    const float dxf = kx * inn;  // el raton esta dxf mm mas adelante de lo que creia
#ifdef RATON_DEBUG
    if (getenv("RADARDBG") && t_ > atof(getenv("RADARDBG")) && t_ < atof(getenv("RADARDBG")) + 1.5)
      printf("%.3f LONG s%d mm=%d h0=%.1f inn=%.1f hx=%.2f hy=%.2f dx=%.2f var=%.1f pos=%.1f\n", t_, sensor, mm, h0, inn,
             hx, hy, dxf, longVar_, cellPos());
#endif
    longVar_ -= kx * hx * longVar_;
    if (!cellLearned_ && sensor == 1 && fabsf(hx) > 0.7f) {
      // Paso de celda: error acumulado desde el inicio del tramo recto repartido entre las celdas que lleva.
      // Tambien en la competencia: la pista real no mide lo nominal y las ruedas tampoco (las dos cosas
      // hacen que la odometria y las celdas no cuadren); en una recta larga serian centimetros.
      const float facePos = cellPos() + p_.frontAheadMm + h0 + 0.5f * p_.wallMm - 0.5f * cell_;
      const int k = static_cast<int>(floorf(facePos / cell_ + 0.5f));
      // Tramos de 2 celdas o mas; en los cortos pesa mas el error con el que se salio del giro anterior.
      const float n = straightCells_ + k + 0.5f;
      if (straightCells_ + k >= 2) {
        cellLearned_ = true;
        const float err = (straightCorr_ - inn / hx) / n;
        const float before = cell_;
        const float gain = learnCell_ && cellUpdates_ == 0 ? 0.6f : 0.35f * clampf(n / 5.0f, 0.3f, 1.0f);
        const float span = learnCell_ ? 0.08f : 0.05f;
        ++cellUpdates_;
        cell_ = clampf(cell_ + gain * err, (1.0f - span) * cellRef_, (1.0f + span) * cellRef_);
#ifdef RATON_DEBUG
        if (getenv("CELLDBG"))
          printf("%.3f PASO err=%.2f celdas=%d+%d corr=%.1f inn=%.1f -> %.1f\n", t_, err, straightCells_, k, straightCorr_,
                 inn / hx, cell_);
#endif
        halfCorr_ = 0.5f * (cell_ - p_.wallMm);
        sonda_.setup(cell_, p_.wallMm, p_.sensorFovDeg, p_.sensorMaxMm);
        if (fabsf(cell_ - before) >= 1.0f) {
          char line[40];
          snprintf(line, sizeof(line), "PASO_CELDA %d mm", static_cast<int>(cell_ + 0.5f));
          io_.report(line, false);
        }
      }
    }
    applyCorrection(-dxf, 1.0f);
  }
}

void Navigator::learnVertical(int sensor, int mm, float h0) {
  // Inclinacion real de cada sensor (y su altura) y reflejo del piso: deciden que parte del cono toca el piso
  // o pasa por encima de las paredes, y a mano no se miden bien. Se ajustan solos con las lecturas que el
  // mapa ya explica (paredes seguras, pose bien conocida, en recto), poco a poco: media de muchas lecturas.
  if (!p_.learnVertical || mm < 0 || h0 < 0 || motion_.turning() || fabsf(headingError()) > 3.0f) return;
  if (longVar_ > 30.0f || lateral_.varY() > 25.0f) return;
  if (!confident(scan_, static_cast<float>(sonda_.maxMm()))) return;
  const float t0 = sonda_.tiltDeg(sensor), f0 = sonda_.floorReflect();
  sonda_.setTilt(sensor, t0 + 0.5f);
  const float pt = sonda_.predict(maze_, scan_);
  sonda_.setTilt(sensor, t0);
  sonda_.setFloorReflect(f0 * 1.2f);
  const float pf = sonda_.predict(maze_, scan_);
  sonda_.setFloorReflect(f0);
  if (pt < 0 || pf < 0) return;
  const float gt = (pt - h0) * 2.0f;    // mm por grado
  const float gf = (pf - h0) * 5.485f;  // mm por unidad de ln(reflejo) (1 / ln 1.2)
  const float g2 = gt * gt + gf * gf;
  if (g2 < 9.0f) return;  // esta lectura casi no depende de ellos (pared cerca de frente)
  // Gradiente normalizado: cada lectura mueve poco; el ruido y los errores de pose se promedian.
  const float e = clampf(static_cast<float>(mm) - h0, -25.0f, 25.0f);
  const float k = 0.02f * e / (g2 + 25.0f);
  sonda_.setTilt(sensor, clampf(t0 + k * gt, p_.sensorTiltDeg - 8.0f, p_.sensorTiltDeg + 8.0f));
  sonda_.setFloorReflect(clampf(f0 * expf(k * gf), 0.03f, 1.0f));
}

bool Navigator::frontSeesWall(int k, int mm) {
  // Con el sensor a cierta altura, el frontal ve el piso a unos centimetros (y por encima de las paredes lo
  // de fuera): una lectura corta no siempre es una pared. Se compara con la sonda: que leeria con la pared
  // de la frontera k (0 = salida de esta celda) y sin ella (piso, paredes lejanas segun el mapa).
  if (mm < 0) return false;
  const int x = cx_ + k * dx(h_), y = cy_ + k * dy(h_);
  if (!maze_.inside(x, y)) return true;
  int seg;
  switch (h_) {
    case NORTH: seg = Maze::hSeg(x, y + 1); break;
    case SOUTH: seg = Maze::hSeg(x, y); break;
    case EAST: seg = Maze::vSeg(x + 1, y); break;
    default: seg = Maze::vSeg(x, y); break;
  }
  if (maze_.segBorder(seg)) return true;
  float sx, sy, ang;
  sensorPose(1, 0, 0, 0, sx, sy, ang);
  sonda_.trace(maze_, sx, sy, ang, scanJ_);
  Sonda::Override ov;
  ov.set(seg, true);
  const float pc = sonda_.predict(maze_, scanJ_, &ov);
  ov.set(seg, false);
  float po = sonda_.predict(maze_, scanJ_, &ov);
  if (pc < 0) return false;
  if (po < 0) po = static_cast<float>(sonda_.maxMm()) + 100.0f;
  // Sin distancia fija alrededor de pc: justo cuando se usa esto (pared que no se esperaba, atasco) el avance
  // puede estar mal. Basta con que la lectura este mas cerca de "pared" que de "sin pared".
  if (po - pc < 15.0f) return true;  // la sonda no distingue (rumbo raro): como antes
  return static_cast<float>(mm) < 0.5f * (pc + po);
}

void Navigator::frontCheck(int mm) {
  // Pared justo delante que no deberia estar (el plan era pasar): parar antes de tocarla.
  if (mm < 0 || motion_.turning() || fabsf(headingError()) > 6.0f) {
    blockedHits_ = 0;
    return;
  }
  const float sensorRel = cellPos() - motion_.measuredSpeed() * p_.sensorLatencyS + p_.frontAheadMm;
  const float halfWall = 0.5f * p_.wallMm;
  const float calib = frontStopReading() - (0.5f * cell_ - halfWall - p_.frontAheadMm);
  const float dist = static_cast<float>(mm) - calib;              // del sensor a la cara
  const float residual = sensorRel + dist + halfWall - 0.5f * cell_;  // respecto a la frontera de esta celda
  const bool passing = !decided_ || act_ == Act::Forward || !(cx_ == actX_ && cy_ == actY_);
  const float vv = motion_.speed();
  const float stopDist = vv * vv / (2.0f * p_.decel);
  if (fabsf(residual) < 0.3f * cell_ && passing && dist < frontStopReading() + stopDist + 30.0f &&
      frontSeesWall(0, mm)) {
    if (++blockedHits_ >= 2) {
      blocked_ = true;
      maze_.observe(cx_, cy_, h_, true, 2);
    }
  } else {
    blockedHits_ = 0;
  }
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
  // 2) Empujando una pared con las ruedas patinando: los encoders avanzan pero el frontal no se acerca. Solo
  //    si lo que ve el frontal es una pared (el piso o lo lejano no se acercan aunque el raton avance): mas
  //    cerca de lo que empieza el piso y explicado por una pared en el mapa.
  const float floorStart = sonda_.floorStartMm(1);
  const float pushMax = floorStart > 0 && 0.8f * floorStart < 250.0f ? 0.8f * floorStart : 250.0f;
  if (motion_.turning() || motion_.speed() < 40.0f || s_.front.mm < 0 || s_.front.mm > pushMax) {
    pushRefOdo_ = -1e9f;
    return;
  }
  if (!s_.front.fresh) return;
  if (!frontSeesWall(0, s_.front.mm) && !frontSeesWall(1, s_.front.mm)) {
    pushRefOdo_ = -1e9f;
    return;
  }
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
  // Pose estimada en el marco del laberinto: x, y = eje en mm (x este, y norte, desde la esquina de la celda
  // 0,0), a = rumbo de brujula (grados, 0 = norte, + = izquierda), P = paso de celda x10, H = rumbo x10
  // (0 = este), G = 1 en tramo recto.
  const float pos = cellPos(), yl = lateral_.y();
  const float xmm = (cx_ + 0.5f) * cell_ + pos * dx(h_) - yl * dy(h_);
  const float ymm = (cy_ + 0.5f) * cell_ + pos * dy(h_) + yl * dx(h_);
  const float head = atan2f(static_cast<float>(dy(h_)), static_cast<float>(dx(h_))) * kDeg + headingError();
  char line[140];
  snprintf(line, sizeof(line), "NAV I%d F%d D%d v%d r%d p%d x%d y%d a%d P%d H%d G%d", s_.left.mm, s_.front.mm,
           s_.right.mm, static_cast<int>(motion_.measuredSpeed()), static_cast<int>(wrap180(motion_.angle())),
           static_cast<int>(cell_ - p_.wallMm), static_cast<int>(xmm), static_cast<int>(ymm),
           static_cast<int>(wrap180(head - 90.0f)), static_cast<int>(cell_ * 10.0f),
           static_cast<int>(wrap180(head) * 10.0f), motion_.turning() ? 0 : 1);
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
      planVEnd_ = planArc_ ? arcSpeed() : 0.0f;
    } else {
      planActive_ = false;
      motion_.cruise(cruiseSpeed(nx, ny));
    }
  } else if ((act_ == Act::Left || act_ == Act::Right) && smooth_ && smoothOk_ &&
             motion_.position() < cCenter - smoothDf_ - 5.0f) {
    planArc_ = true;
    planVEnd_ = arcSpeed();
    arcPlanned_ = true;
  } else {
    planVEnd_ = 0;
  }
  if (planActive_) motion_.moveTo(planTarget(), vmax_, planVEnd_);
  arcStarted_ = false;
}

float Navigator::cruiseSpeed(int nx, int ny) const {
  // Explorando, por celdas ya recorridas (paredes conocidas) y en recta, se puede ir mas rapido: no hay nada
  // que mirar con calma y la parada siguiente se decide con tiempo para frenar. En lo nuevo, vmax.
  const int fx = nx + dx(h_), fy = ny + dy(h_);
  if (smooth_ || p_.knownSpeed <= vmax_ || !maze_.visited(nx, ny) || !maze_.visited(fx, fy)) return vmax_;
  if (maze_.votes(nx, ny, h_) > -3) return vmax_;  // la salida de la celda siguiente, confirmada abierta
  return p_.knownSpeed;
}

float Navigator::arcSpeed(float radius) const {
  // En la curva la rueda de fuera va mas rapido que el centro: que no pase de lo que dan los motores (con
  // bateria baja o motores lentos la curva se abriria y chocaria).
  const float outer = 1.0f + 0.5f * p_.trackMm / (radius > 0 ? radius : smoothRadius_);
  const float cap = 0.85f * motion_.topSpeed() / outer;
  return p_.turnSpeed < cap ? p_.turnSpeed : cap;
}

bool Navigator::waitStopped(float timeout, float speedTol) {
  const float t0 = t_;
  while (t_ - t0 < timeout) {
    tick();
    if (io_.stopRequested()) return false;
    if (motion_.arrived() && !motion_.turning() && fabsf(motion_.measuredSpeed()) < speedTol &&
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
#ifdef RATON_DEBUG
  if (getenv("PIVDBG"))
    printf("%.3f PIVOTE q=%d y=%.1f varY=%.1f pos=%.1f ang=%.1f target=%.1f\n", t_, q, lateral_.y(), lateral_.varY(),
           cellPos(), motion_.angle(), target);
#endif
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
  longVar_ += lateral_.varY() + 4.0f;  // el avance nuevo es el lateral de antes
  resetBeams();
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
  // El giro acaba a menos de 3 grados: terminar de alinearse con la cuadricula (parado, sin prisa).
  motion_.turnTo(gridAngle_, 0.3f * p_.rotSpeed, p_.rotAccel);
  waitStopped(0.5f);
}

void Navigator::aim(float deg) {
  motion_.turnTo(gridAngle_ + deg, 0.5f * p_.rotSpeed, p_.rotAccel);
  waitStopped(1.2f);
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
  const bool wall = mm >= 0 && mm < frontStopReading() + 0.45f * cell_ && frontSeesWall(0, mm);
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
  if (!frontSeesWall(k, f)) f = -1;  // lo que ve no es la pared (piso, lejos): no moverse por ello
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
  if (f >= 0) longVar_ = 9.0f;
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
    // Pared lateral plana: lectura = distancia perpendicular x factor del cono.
    const float g = sonda_.parallelFactor(p_.diagMountDeg / kDeg);
    const float latL = l >= 0 ? p_.diagLateralMm + l / g : 1e9f;
    const float latR = r >= 0 ? p_.diagLateralMm + r / g : 1e9f;
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
  if (f >= 0 && f < 0.5f * cell_ + 40.0f && frontSeesWall(0, f)) {
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
      longVar_ += 100.0f;
      resetBeams();
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
      // Casi parado basta para empezar a girar: los ultimos mm/s tardan en irse y el giro los absorbe.
      waitStopped(0.4f, 40.0f);
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
