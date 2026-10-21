#include "sim.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <algorithm>
#include <fstream>

#include "raton/maze_text.h"

namespace sim {

using raton::Dir;
using raton::Maze;
using raton::Wall;

namespace {
const float kPi = 3.14159265f;
const float kDeg = 57.2957795f;
}  // namespace

// ---------------------------------------------------------------------------------------------
// Mundo
// ---------------------------------------------------------------------------------------------

World::World(const Maze& truth, const Physical& ph) : truth_(truth), ph_(ph) {
  const float c = ph.cellMm, hw = 0.5f * ph.wallMm;
  const int w = truth.width(), h = truth.height();
  // Postes en todas las esquinas.
  for (int y = 0; y <= h; ++y)
    for (int x = 0; x <= w; ++x) rects_.push_back({x * c - hw, y * c - hw, x * c + hw, y * c + hw});
  for (int y = 0; y < h; ++y)
    for (int x = 0; x < w; ++x) {
      if (truth.wall(x, y, raton::SOUTH) == Wall::Closed)
        rects_.push_back({x * c, y * c - hw, (x + 1) * c, y * c + hw});
      if (truth.wall(x, y, raton::WEST) == Wall::Closed)
        rects_.push_back({x * c - hw, y * c, x * c + hw, (y + 1) * c});
      if (y == h - 1 && truth.wall(x, y, raton::NORTH) == Wall::Closed)
        rects_.push_back({x * c, (y + 1) * c - hw, (x + 1) * c, (y + 1) * c + hw});
      if (x == w - 1 && truth.wall(x, y, raton::EAST) == Wall::Closed)
        rects_.push_back({(x + 1) * c - hw, y * c, (x + 1) * c + hw, (y + 1) * c});
    }
}

float gFovDeg = 25.0f;
bool gAutoCal = true;
float gOutsideMm = 0.0f;

float World::raycast(float x, float y, float ang, float maxd, int* axis) const {
  const float ddx = cosf(ang), ddy = sinf(ang);
  float best = maxd;
  int bestAxis = 0;
  for (const Rect& r : rects_) {
    // Interseccion rayo-caja (slabs).
    float t0 = 0, t1 = best;
    int ax = 0;
    if (fabsf(ddx) < 1e-9f) {
      if (x < r.x0 || x > r.x1) continue;
    } else {
      float a = (r.x0 - x) / ddx, b = (r.x1 - x) / ddx;
      if (a > b) std::swap(a, b);
      t0 = std::max(t0, a);
      t1 = std::min(t1, b);
      if (t0 > t1) continue;
    }
    if (fabsf(ddy) < 1e-9f) {
      if (y < r.y0 || y > r.y1) continue;
    } else {
      float a = (r.y0 - y) / ddy, b = (r.y1 - y) / ddy;
      if (a > b) std::swap(a, b);
      if (a > t0) ax = 1;
      t0 = std::max(t0, a);
      t1 = std::min(t1, b);
      if (t0 > t1) continue;
    }
    if (t0 < best) {
      best = t0;
      bestAxis = ax;
    }
  }
  if (axis) *axis = bestAxis;
  return best;
}

float World::coneReading(float x, float y, float ang, float fovDeg) const {
  // Cono en 3D: columnas repartidas en el angulo horizontal y, en cada una, rayos repartidos en vertical
  // (seccion circular del cono). Cada rayo toca la primera cara que encuentra a su altura (las paredes
  // miden wallHeightMm), el piso, o pasa por encima de todo y llega a lo de fuera. Peso = luz que vuelve
  // (coseno de incidencia / distancia^2, x reflejo). La lectura es la media pesada; con poca senal, 2000.
  const int kCols = 13, kRows = 7;
  const float half = 0.5f * fovDeg / kDeg;
  const float h = ph_.sensorHeightMm, H = ph_.wallHeightMm, tilt = ph_.sensorTiltDeg / kDeg;
  float sw = 0, swd = 0;
  struct Face {
    float t;
    int axis;
  };
  Face faces[24];
  for (int j = 0; j < kCols; ++j) {
    const float f = -1.0f + (2.0f * j + 1.0f) / kCols;  // -1..1 dentro del cono
    const float a = ang + f * half;
    const float ddx = cosf(a), ddy = sinf(a);
    // Todas las caras que cruza en horizontal, de la mas cercana a la mas lejana.
    int n = 0;
    for (const Rect& r : rects_) {
      float t0 = 0, t1 = 2000.0f;
      int ax = 0;
      if (fabsf(ddx) < 1e-9f) {
        if (x < r.x0 || x > r.x1) continue;
      } else {
        float p = (r.x0 - x) / ddx, q = (r.x1 - x) / ddx;
        if (p > q) std::swap(p, q);
        t0 = std::max(t0, p);
        t1 = std::min(t1, q);
        if (t0 > t1) continue;
      }
      if (fabsf(ddy) < 1e-9f) {
        if (y < r.y0 || y > r.y1) continue;
      } else {
        float p = (r.y0 - y) / ddy, q = (r.y1 - y) / ddy;
        if (p > q) std::swap(p, q);
        if (p > t0) ax = 1;
        t0 = std::max(t0, p);
        t1 = std::min(t1, q);
        if (t0 > t1) continue;
      }
      if (n < 24) faces[n++] = {t0, ax};
    }
    std::sort(faces, faces + n, [](const Face& p, const Face& q) { return p.t < q.t; });
    const float chord = sqrtf(1.0f - f * f);
    for (int k = 0; k < kRows; ++k) {
      const float g = -1.0f + (2.0f * k + 1.0f) / kRows;
      const float e = tilt + g * half * chord;
      const float ce = cosf(e), te = tanf(e);
      const float sFloor = e < 0 ? h / -te : 1e9f;
      float range = -1, inc = 0, refl = 1;
      for (int q = 0; q < n && range < 0; ++q) {
        if (faces[q].t >= sFloor) break;
        if (h + faces[q].t * te <= H) {
          range = faces[q].t / ce;
          inc = (faces[q].axis == 0 ? fabsf(ddx) : fabsf(ddy)) * ce;
        }
      }
      if (range < 0 && sFloor < 2000.0f) {
        range = sFloor / ce;
        inc = fabsf(sinf(e));
        refl = ph_.floorReflect;
      }
      if (range < 0 && ph_.outsideMm > 0) {
        range = ph_.outsideMm;
        inc = 1.0f;
      }
      if (range < 0) continue;
      const float dd = std::max(range, 10.0f);
      const float w = chord / kRows * inc * refl / (dd * dd);
      sw += w;
      swd += w * range;
    }
  }
  // Umbral: una pared de frente que llena el cono a 1200 mm (media de sqrt(1 - f^2) = pi / 4).
  const float ref = 0.785f / (1200.0f * 1200.0f);
  return sw / kCols >= ref ? swd / sw : 2000.0f;
}

bool World::collidesBody(float x, float y, float th, float front, float rear, float w) const {
  // Separacion de ejes entre el rectangulo girado del chasis y cada caja (paredes y postes).
  const float c = cosf(th), s = sinf(th);
  const float cx = x + 0.5f * (front - rear) * c, cy = y + 0.5f * (front - rear) * s;
  const float hl = 0.5f * (front + rear), hw = 0.5f * w;
  const float ex = fabsf(hl * c) + fabsf(hw * s), ey = fabsf(hl * s) + fabsf(hw * c);  // caja que lo contiene
  for (const Rect& r : rects_) {
    if (cx + ex < r.x0 || cx - ex > r.x1 || cy + ey < r.y0 || cy - ey > r.y1) continue;
    // Ejes del chasis: proyectar la caja.
    const float rx = 0.5f * (r.x0 + r.x1) - cx, ry = 0.5f * (r.y0 + r.y1) - cy;
    const float bx = 0.5f * (r.x1 - r.x0), by = 0.5f * (r.y1 - r.y0);
    const float along = rx * c + ry * s, across = -rx * s + ry * c;
    if (fabsf(along) > hl + bx * fabsf(c) + by * fabsf(s)) continue;
    if (fabsf(across) > hw + bx * fabsf(s) + by * fabsf(c)) continue;
    return true;
  }
  return false;
}

bool World::collides(float x, float y, float rad) const {
  for (const Rect& r : rects_) {
    const float cx = std::max(r.x0, std::min(x, r.x1));
    const float cy = std::max(r.y0, std::min(y, r.y1));
    if ((cx - x) * (cx - x) + (cy - y) * (cy - y) < rad * rad) return true;
  }
  return false;
}

// ---------------------------------------------------------------------------------------------
// Plataforma simulada
// ---------------------------------------------------------------------------------------------

SimPlatform::SimPlatform(const World& world, const raton::Params& p, const Physical& ph, uint32_t seed)
    : world_(world), p_(p), ph_(ph), rng_(seed) {
  std::normal_distribution<float> n(0, 1);
  bias_ = ph_.gyroBiasDps * (n(rng_) > 0 ? 1.0f : -1.0f);
  for (int i = 0; i < 5; ++i) laserReady_[i] = 0.005f * i;
}

void SimPlatform::place(int cx, int cy, int dir) {
  const float c = ph_.cellMm;
  x_ = (cx + 0.5f) * c + ph_.startX;
  y_ = (cy + 0.5f) * c + ph_.startY;
  th_ = (90.0f - 90.0f * dir + ph_.startDeg) / kDeg;
  wl_ = wr_ = bl_ = br_ = 0;
  dutyL_ = dutyR_ = 0;
  touching_ = false;
}

float SimPlatform::headingDeg() const { return th_ * kDeg; }
int SimPlatform::cellX() const { return static_cast<int>(floorf(x_ / ph_.cellMm)); }
int SimPlatform::cellY() const { return static_cast<int>(floorf(y_ / ph_.cellMm)); }

void SimPlatform::setMotors(float l, float r) {
  dutyL_ = std::max(-1.0f, std::min(1.0f, l));
  dutyR_ = std::max(-1.0f, std::min(1.0f, r));
}

void SimPlatform::physics(float dt) {
  auto target = [&](float d, float gain) {
    const float a = fabsf(d) - ph_.stictionDuty;
    if (a <= 0) return 0.0f;
    return (d > 0 ? 1.0f : -1.0f) * a * ph_.vmaxFull * gain;
  };
  const float tl = target(dutyL_, 1.0f - 0.5f * ph_.motorAsym);
  const float tr = target(dutyR_, 1.0f + 0.5f * ph_.motorAsym);
  wl_ += (tl - wl_) * dt / ph_.motorTau;
  wr_ += (tr - wr_) * dt / ph_.motorTau;
  // Adherencia: el suelo sigue a la rueda como mucho a "grip".
  auto follow = [&](float b, float w) {
    const float step = ph_.grip * dt;
    if (w > b + step) return b + step;
    if (w < b - step) return b - step;
    return w;
  };
  bl_ = follow(bl_, wl_);
  br_ = follow(br_, wr_);
  // Encoder: lo que gira la rueda (con su diametro real; la logica usa el nominal).
  encL_ += wl_ * dt;
  encR_ += wr_ * dt;
  const float v = 0.5f * (bl_ + br_);
  const float w = (br_ - bl_) / ph_.trackMm;
  const float nth = th_ + w * dt;
  const float nx = x_ + v * cosf(th_ + 0.5f * w * dt) * dt;
  const float ny = y_ + v * sinf(th_ + 0.5f * w * dt) * dt;
  const float rad = 0.5f * ph_.bodyDiamMm;
  auto hits = [&](float px, float py, float pth) {
    return ph_.rectBody ? world_.collidesBody(px, py, pth, ph_.bodyFrontMm, ph_.bodyRearMm, ph_.bodyWidthMm)
                        : world_.collides(px, py, rad);
  };
  if (hits(nx, ny, nth)) {
    // Contra la pared: puede girar en el sitio si no choca, pero no avanzar.
    if (!touching_) {
      ++contacts_;
      if (getenv("SIM_CONTACTLOG"))
        printf("CONTACTO t=%.2f v=%.0f w=%.0f err=%.0f/%.0f/%.1f\n", t_, v, w * kDeg, lastErr_[0], lastErr_[1], lastErr_[2]);
      if (verbose) printf("%8.3f  !! choque en %.0f,%.0f rumbo %.0f v=%.0f\n", t_, x_, y_, th_ * kDeg, v);
    }
    touching_ = true;
    contactTime_ += dt;
    if (!hits(x_, y_, nth)) th_ = nth;
    bl_ = br_ = 0;
    wl_ *= 0.2f;  // la rueda se frena contra la pared (el encoder casi no cuenta)
    wr_ *= 0.2f;
  } else {
    touching_ = false;
    x_ = nx;
    y_ = ny;
    th_ = nth;
  }
  yawTrue_ += w * dt * kDeg;
}

int SimPlatform::readLaser(int i) {
  const float c = cosf(th_), s = sinf(th_);
  float fx, fy, ang;
  const bool cone = ph_.fovDeg > 0;
  if (i == 1) {
    fx = p_.frontAheadMm;
    fy = 0;
    ang = 0;
  } else if (i >= 3) {
    const float sign = i == 3 ? 1.0f : -1.0f;
    fx = p_.sideAheadMm;
    fy = sign * p_.sideLateralMm;
    ang = sign * 90.0f / kDeg;
  } else {
    const float sign = i == 0 ? 1.0f : -1.0f;
    fx = p_.diagAheadMm;
    fy = sign * p_.diagLateralMm;
    ang = sign * (cone ? ph_.mountDeg : ph_.diagAngleDeg) / kDeg;
  }
  const float sx = x_ + fx * c - fy * s, sy = y_ + fx * s + fy * c;
  const float d = cone ? world_.coneReading(sx, sy, th_ + ang, ph_.fovDeg) : world_.raycast(sx, sy, th_ + ang, 2000.0f);
  std::uniform_real_distribution<float> u(0, 1);
  std::normal_distribution<float> n(0, 1);
  if (d > 1200.0f) return -1;
  if (u(rng_) < ph_.dropout) return -1;
  if (u(rng_) < ph_.outlier) return static_cast<int>(20 + u(rng_) * 700);
  const int off = i >= 3 ? 0 : (i == 0 ? ph_.offsetL : (i == 1 ? ph_.offsetF : ph_.offsetR));
  float v = d + off + n(rng_) * (ph_.rangeNoiseMm + ph_.rangeNoise * d);
  if (v < 0) v = 0;
  return static_cast<int>(v + 0.5f);
}

void SimPlatform::updateLasers() {
  // Cada sensor mide en continuo (~21 ms). El valor es el de la mitad de la medida, que esta lista
  // ~10 ms despues; la placa lee un sensor por ciclo (por turnos).
  for (int i = 0; i < (ph_.sideSensors ? 5 : 3); ++i) {
    if (!laserPending_[i] && t_ >= laserReady_[i] - 0.0105f) {
      laserValue_[i] = readLaser(i);
      laserPending_[i] = true;
    }
  }
}

const raton::Sensors& SimPlatform::tick() {
  const float dt = p_.dt;
  const int sub = 4;
  for (int k = 0; k < sub; ++k) {
    physics(dt / sub);
    t_ += dt / sub;
    updateLasers();
  }
  ++tickCount_;
  s_.t = t_;
  const float cpr = p_.countsPerRev;
  // Cuentas enteras con el diametro real, convertidas con el nominal (como el firmware).
  const long cl = static_cast<long>(encL_ / (kPi * ph_.wheelDiamL) * cpr);
  const long cr = static_cast<long>(encR_ / (kPi * ph_.wheelDiamR) * cpr);
  s_.encLeftMm = cl * kPi * p_.wheelDiamMm / cpr;
  s_.encRightMm = cr * kPi * p_.wheelDiamMm / cpr;
  // Giroscopio a 200 Hz.
  std::normal_distribution<float> n(0, 1);
  s_.gyroFresh = false;
  if (t_ >= nextGyro_) {
    nextGyro_ += 0.005f;
    bias_ += ph_.gyroDriftDps * sqrtf(0.005f) * n(rng_);
    const float wTrue = (br_ - bl_) / ph_.trackMm * kDeg;
    const float rate = wTrue * ph_.gyroScale + bias_ + ph_.gyroNoiseDps * n(rng_);
    const bool failed = ph_.gyroFailAt >= 0 && t_ >= ph_.gyroFailAt;
    if (!failed) s_.gyroDeg += rate * (t_ - lastGyroRead_);
    lastGyroRead_ = t_;
    s_.gyroFresh = !failed || ph_.gyroFrozen;
  }
  // Laseres por turnos.
  raton::Range* rr[5] = {&s_.left, &s_.front, &s_.right, &s_.sideLeft, &s_.sideRight};
  const int offs[5] = {p_.leftOffsetMm, p_.frontOffsetMm, p_.rightOffsetMm,
                       p_.sideLeftOffsetMm, p_.sideRightOffsetMm};
  for (int i = 0; i < 5; ++i) rr[i]->fresh = false;
  const int i = tickCount_ % (ph_.sideSensors ? 5 : 3);
  if (laserPending_[i] && t_ >= laserReady_[i]) {
    std::uniform_real_distribution<float> u(0, 1);
    laserPending_[i] = false;
    laserReady_[i] += 0.021f + 0.002f * u(rng_);
    if (laserReady_[i] < t_) laserReady_[i] = t_ + 0.021f;
    int v = laserValue_[i];
    if (v >= 0) v += offs[i];
    if (v > p_.sensorMaxMm) v = -1;
    if (!(ph_.laserFailAt[i] >= 0 && t_ >= ph_.laserFailAt[i])) {
      rr[i]->mm = v;
      rr[i]->fresh = true;
    }
  }
  // Zona real (regla de los 30 s).
  const int cx = cellX(), cy = cellY();
  if (abs(cx - ax_) + abs(cy - ay_) > 1) {
    ax_ = cx;
    ay_ = cy;
    anchorT_ = t_;
  } else if (t_ - anchorT_ > maxArea_) {
    maxArea_ = t_ - anchorT_;
  }
  if (watch) {
    const raton::Maze& tm = world_.truth();
    for (int y = 0; y < tm.height(); ++y)
      for (int x = 0; x < tm.width(); ++x) {
        uint8_t st = 0;
        for (int d = 0; d < 2; ++d) {
          const Dir dir = d == 0 ? raton::NORTH : raton::EAST;
          const Wall k = watch->wall(x, y, dir);
          if (k != Wall::Unknown && k != tm.wall(x, y, dir)) st |= static_cast<uint8_t>(1 << d);
        }
        uint8_t& old = wrongState_[tm.index(x, y)];
        if (st & ~old) {
          ++wrongEvents;
          if (verbose)
            printf("%8.3f  !! pared mal x=%d y=%d %s (mapa=%s) robot en %.0f,%.0f rumbo %.0f\n", t_, x, y,
                   (st & ~old & 1) ? "N" : "E", watch->wall(x, y, (st & ~old & 1) ? raton::NORTH : raton::EAST) == Wall::Closed ? "pared" : "abierta",
                   x_, y_, th_ * kDeg);
        }
        old = st;
      }
  }
  if (trace && tickCount_ % 50 == 0)
    printf("%7.2f pose %.0f %.0f %.0f duty %.2f %.2f L%d F%d R%d\n", t_, x_, y_, th_ * kDeg, dutyL_, dutyR_, s_.left.mm,
           s_.front.mm, s_.right.mm);
  return s_;
}

void SimPlatform::report(const char* line, bool important) {
  if (!strncmp(line, "NAV ", 4)) {
    // Error de la pose estimada frente a la real, en el marco del raton (avance / lateral / rumbo).
    const char* px = strstr(line, " x");
    const char* py = strstr(line, " y");
    const char* pc = strstr(line, " P");
    const char* ph = strstr(line, " H");
    const char* pg = strstr(line, " G");
    if (px && py && pc && ph && pg && pg[2] == '1') {  // solo en tramos rectos (girando la pose es otra cosa)
      const float cellB = static_cast<float>(atoi(pc + 2)) / 10.0f;
      const float s = ph_.cellMm / cellB;  // la pose estimada en celdas, pasada a mm reales
      const float ex = atoi(px + 2) * s - x_, ey = atoi(py + 2) * s - y_;
      float eh = static_cast<float>(atoi(ph + 2)) / 10.0f - th_ * kDeg;
      while (eh > 180) eh -= 360;
      while (eh < -180) eh += 360;
      const float c = cosf(th_), sn = sinf(th_);
      const float el = ex * c + ey * sn, et = -ex * sn + ey * c;
      if (getenv("SIM_LOCLOG"))
        printf("%8.3f LOC err=%.0f/%.0f/%.1f real=%.0f,%.0f,%.0f\n", t_, el, et, eh, x_, y_, th_ * kDeg);
      // Rumbo real respecto a la cuadricula y distancia real al centro del pasillo (zigzag y "se pega").
      {
        float a = th_ * kDeg;
        a -= 90.0f * floorf(a / 90.0f + 0.5f);
        const float c = ph_.cellMm;
        const bool alongX = fabsf(cosf(th_)) > 0.7f;
        const float lat = alongX ? y_ - (floorf(y_ / c) + 0.5f) * c : x_ - (floorf(x_ / c) + 0.5f) * c;
        wob2_ += a * a;
        off2_ += lat * lat;
        if (fabsf(lat) > offMax_) offMax_ = fabsf(lat);
      }
      lastErr_[0] = el;
      lastErr_[1] = et;
      lastErr_[2] = eh;
      locN_ += 1;
      locLong2_ += el * el;
      locLat2_ += et * et;
      locHead2_ += eh * eh;
      if (fabsf(el) > locLongMax_) locLongMax_ = fabsf(el);
      if (fabsf(et) > locLatMax_) locLatMax_ = fabsf(et);
      if (fabsf(eh) > locHeadMax_) locHeadMax_ = fabsf(eh);
      // Celda creida distinta de la real (lejos de las fronteras, donde no hay duda): perdido.
      const float bx = atoi(px + 2) / cellB, by = atoi(py + 2) / cellB;
      const float fx = bx - floorf(bx), fy = by - floorf(by);
      if (fx > 0.25f && fx < 0.75f && fy > 0.25f && fy < 0.75f) {
        const bool lost = static_cast<int>(floorf(bx)) != cellX() || static_cast<int>(floorf(by)) != cellY();
        if (lost && !lost_) {
          ++lostEvents_;
          if (verbose)
            printf("%8.3f  !! PERDIDO cree (%d,%d) esta en (%d,%d) error avance %.0f lateral %.0f rumbo %.1f\n", t_,
                   static_cast<int>(floorf(bx)), static_cast<int>(floorf(by)), cellX(), cellY(), el, et, eh);
        }
        lost_ = lost;
      }
    }
  }
  if (!important) return;
  events.push_back({t_, line});
  if (verbose) printf("%8.3f  %s\n", t_, line);
}

bool SimPlatform::saveMap(const raton::Maze& maze) {
  (void)maze;
  saved = true;
  return true;
}

// ---------------------------------------------------------------------------------------------
// Laberintos
// ---------------------------------------------------------------------------------------------

bool loadMazeFile(const std::string& path, Maze& out, std::vector<std::string>* rowsOut) {
  std::ifstream f(path);
  if (!f) return false;
  std::vector<std::string> rows;
  std::string line;
  while (std::getline(f, line)) {
    while (!line.empty() && (line.back() == '\r' || line.back() == ' ')) line.pop_back();
    if (line.empty() || line[0] == '#') continue;
    rows.push_back(line);
  }
  // Las filas de paredes pueden acabar antes por los espacios recortados: rellenar.
  size_t len = 0;
  for (auto& r : rows) len = std::max(len, r.size());
  for (auto& r : rows) r.resize(len, ' ');
  std::vector<const char*> ptrs;
  for (auto& r : rows) ptrs.push_back(r.c_str());
  if (rowsOut) *rowsOut = rows;
  return raton::parseMazeText(ptrs.data(), static_cast<int>(ptrs.size()), out);
}

void randomMaze(int w, int h, int loops, uint32_t seed, Maze& out) {
  std::mt19937 rng(seed);
  out.reset(w, h, true);
  for (int y = 0; y < h; ++y)
    for (int x = 0; x < w; ++x)
      for (int d = 0; d < 4; ++d) out.set(x, y, static_cast<Dir>(d), Wall::Closed);
  std::vector<int> stack;
  std::vector<bool> seen(w * h, false);
  stack.push_back(0);
  seen[0] = true;
  while (!stack.empty()) {
    const int c = stack.back();
    const int x = c % w, y = c / w;
    int opts[4], n = 0;
    for (int d = 0; d < 4; ++d) {
      const int nx = x + raton::dx(static_cast<Dir>(d)), ny = y + raton::dy(static_cast<Dir>(d));
      if (out.inside(nx, ny) && !seen[ny * w + nx]) opts[n++] = d;
    }
    if (!n) {
      stack.pop_back();
      continue;
    }
    const Dir d = static_cast<Dir>(opts[rng() % n]);
    out.set(x, y, d, Wall::Open);
    const int nx = x + raton::dx(d), ny = y + raton::dy(d);
    seen[ny * w + nx] = true;
    stack.push_back(ny * w + nx);
  }
  for (int i = 0; i < loops; ++i) {
    const int x = 1 + static_cast<int>(rng() % (w - 2)), y = 1 + static_cast<int>(rng() % (h - 2));
    out.set(x, y, static_cast<Dir>(rng() % 4), Wall::Open);
  }
}

void makeStartCell(Maze& m, int x, int y, raton::Dir d, uint32_t seed) {
  // Salida de competencia: pared a la izquierda y a la derecha, abierta al frente. Si eso deja celdas
  // aisladas, se abre una pared entre lo aislado y lo alcanzable hasta que todo vuelve a estar conectado.
  m.set(x, y, raton::turnLeft(d), Wall::Closed);
  m.set(x, y, raton::turnRight(d), Wall::Closed);
  m.set(x, y, d, Wall::Open);
  std::mt19937 rng(seed * 17u + 3u);
  for (int guard = 0; guard < 1000; ++guard) {
    std::vector<bool> seen(m.width() * m.height(), false);
    std::vector<int> q{m.index(x, y)};
    seen[q[0]] = true;
    for (size_t i = 0; i < q.size(); ++i) {
      const int cx = q[i] % m.width(), cy = q[i] / m.width();
      for (int k = 0; k < 4; ++k) {
        const Dir dd = static_cast<Dir>(k);
        const int nx = cx + raton::dx(dd), ny = cy + raton::dy(dd);
        if (!m.inside(nx, ny) || m.wall(cx, cy, dd) != Wall::Open || seen[m.index(nx, ny)]) continue;
        seen[m.index(nx, ny)] = true;
        q.push_back(m.index(nx, ny));
      }
    }
    if (static_cast<int>(q.size()) == m.cells()) return;
    // Paredes entre lo alcanzable y lo aislado (sin tocar la salida).
    std::vector<std::pair<int, int>> cut;
    for (int c : q) {
      const int cx = c % m.width(), cy = c / m.width();
      if (cx == x && cy == y) continue;
      for (int k = 0; k < 4; ++k) {
        const int nx = cx + raton::dx(static_cast<Dir>(k)), ny = cy + raton::dy(static_cast<Dir>(k));
        if (m.inside(nx, ny) && !seen[m.index(nx, ny)] && !(nx == x && ny == y)) cut.push_back({c, k});
      }
    }
    if (cut.empty()) return;
    const auto& e = cut[rng() % cut.size()];
    m.set(e.first % m.width(), e.first / m.width(), static_cast<Dir>(e.second), Wall::Open);
  }
}

}  // namespace sim
