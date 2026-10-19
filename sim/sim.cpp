#include "sim.h"

#include <math.h>
#include <stdio.h>

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

float World::raycast(float x, float y, float ang, float maxd) const {
  const float ddx = cosf(ang), ddy = sinf(ang);
  float best = maxd;
  for (const Rect& r : rects_) {
    // Interseccion rayo-caja (slabs).
    float t0 = 0, t1 = best;
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
      t0 = std::max(t0, a);
      t1 = std::min(t1, b);
      if (t0 > t1) continue;
    }
    if (t0 < best) best = t0;
  }
  return best;
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
  for (int i = 0; i < 3; ++i) laserReady_[i] = 0.005f * i;
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
  if (world_.collides(nx, ny, rad)) {
    // Contra la pared: puede girar en el sitio si no choca, pero no avanzar.
    if (!touching_) {
      ++contacts_;
      if (verbose) printf("%8.3f  !! choque en %.0f,%.0f rumbo %.0f v=%.0f\n", t_, x_, y_, th_ * kDeg, v);
    }
    touching_ = true;
    contactTime_ += dt;
    if (!world_.collides(x_, y_, rad)) th_ = nth;
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
  if (i == 1) {
    fx = p_.frontAheadMm;
    fy = 0;
    ang = 0;
  } else {
    const float sign = i == 0 ? 1.0f : -1.0f;
    fx = p_.diagAheadMm;
    fy = sign * p_.diagLateralMm;
    ang = sign * ph_.diagAngleDeg / kDeg;
  }
  const float sx = x_ + fx * c - fy * s, sy = y_ + fx * s + fy * c;
  const float d = world_.raycast(sx, sy, th_ + ang, 2000.0f);
  std::uniform_real_distribution<float> u(0, 1);
  std::normal_distribution<float> n(0, 1);
  if (d > 1200.0f) return -1;
  if (u(rng_) < ph_.dropout) return -1;
  if (u(rng_) < ph_.outlier) return static_cast<int>(20 + u(rng_) * 700);
  const int off = i == 0 ? ph_.offsetL : (i == 1 ? ph_.offsetF : ph_.offsetR);
  float v = d + off + n(rng_) * (ph_.rangeNoiseMm + ph_.rangeNoise * d);
  if (v < 0) v = 0;
  return static_cast<int>(v + 0.5f);
}

void SimPlatform::updateLasers() {
  // Cada sensor mide en continuo (~21 ms). El valor es el de la mitad de la medida, que esta lista
  // ~10 ms despues; la placa lee un sensor por ciclo (por turnos).
  for (int i = 0; i < 3; ++i) {
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
    s_.gyroDeg += rate * (t_ - lastGyroRead_);
    lastGyroRead_ = t_;
    s_.gyroFresh = true;
  }
  // Laseres por turnos.
  raton::Range* rr[3] = {&s_.left, &s_.front, &s_.right};
  const int offs[3] = {p_.leftOffsetMm, p_.frontOffsetMm, p_.rightOffsetMm};
  for (int i = 0; i < 3; ++i) rr[i]->fresh = false;
  const int i = tickCount_ % 3;
  if (laserPending_[i] && t_ >= laserReady_[i]) {
    std::uniform_real_distribution<float> u(0, 1);
    laserPending_[i] = false;
    laserReady_[i] += 0.021f + 0.002f * u(rng_);
    if (laserReady_[i] < t_) laserReady_[i] = t_ + 0.021f;
    int v = laserValue_[i];
    if (v >= 0) v += offs[i];
    if (v > p_.sensorMaxMm) v = -1;
    rr[i]->mm = v;
    rr[i]->fresh = true;
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

}  // namespace sim
