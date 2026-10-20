#include "raton/sonda.h"

#include <math.h>

namespace raton {

namespace {
const float kBig = 1e9f;
}

void Sonda::Override::set(int s, bool c) {
  for (int i = 0; i < n; ++i)
    if (seg[i] == s) {
      closed[i] = c;
      return;
    }
  if (n < 8) {
    seg[n] = s;
    closed[n] = c;
    ++n;
  }
}

bool Sonda::Override::find(int s, bool& c) const {
  for (int i = 0; i < n; ++i)
    if (seg[i] == s) {
      c = closed[i];
      return true;
    }
  return false;
}

void Sonda::setup(float cellMm, float wallMm, float fovDeg, int maxMm) {
  cell_ = cellMm;
  hw_ = 0.5f * wallMm;
  halfFov_ = 0.5f * fovDeg / 57.2957795f;
  maxMm_ = maxMm;
  float sw = 0;
  for (int j = 0; j < kRays; ++j) {
    const float f = -1.0f + (2.0f * j + 1.0f) / kRays;  // -1..1 dentro del cono
    rayW_[j] = sqrtf(1.0f - f * f);                      // seccion del cono a esa altura
    rayCos_[j] = cosf(f * halfFov_);
    raySin_[j] = sinf(f * halfFov_);
    sw += rayW_[j];
  }
  wRef_ = sw / (1200.0f * 1200.0f);
}

void Sonda::traceRay(const Maze& m, float x0, float y0, float ux, float uy, Ray& r) const {
  // Recorrido de la rejilla frontera a frontera (Amanatides-Woo). Cada frontera es una banda de pared del
  // grosor wallMm con postes en las esquinas: el rayo entra por la cara a t - hw / a. Si en ese punto hay
  // segmento, es ese; dentro de la banda puede ademas tocar el extremo de un poste (rayos casi paralelos).
  r.n = 0;
  const float ax = fabsf(ux), ay = fabsf(uy);
  const float c = cell_;
  int ix = static_cast<int>(floorf(x0 / c)), iy = static_cast<int>(floorf(y0 / c));
  float tx = ax < 1e-6f ? kBig : ((ux > 0 ? (ix + 1) * c : ix * c) - x0) / ux;
  float ty = ay < 1e-6f ? kBig : ((uy > 0 ? (iy + 1) * c : iy * c) - y0) / uy;
  const float dtx = ax < 1e-6f ? kBig : c / ax, dty = ay < 1e-6f ? kBig : c / ay;
  const float limit = 2.0f * static_cast<float>(maxMm_);  // lo lejano aun pesa algo en la media
  const int w = m.width(), h = m.height();
  while (r.n < kMaxHits) {
    const bool vert = tx < ty;
    const float t = vert ? tx : ty;
    const float a = vert ? ax : ay;          // componente perpendicular a la frontera
    const float va = vert ? uy : ux;         // componente a lo largo de la frontera
    const float base = vert ? y0 : x0;
    const float tIn = t - hw_ / a, tOut = t + hw_ / a;
    if (tIn > limit) break;
    const int line = vert ? (ux > 0 ? ix + 1 : ix) : (uy > 0 ? iy + 1 : iy);
    const int lineMax = vert ? w : h, rowMax = vert ? h : w;
    // Coordenada a lo largo de la frontera al entrar en la banda.
    const float vIn = base + tIn * va;
    const int row = static_cast<int>(floorf(vIn / c));
    const float u = vIn - row * c;
    Hit& hit = r.hits[r.n++];
    hit.t = tIn > 0 ? tIn : 0;
    hit.cosInc = a;
    if (u < hw_ || u > c - hw_ || row < 0 || row >= rowMax || line <= 0 || line >= lineMax) {
      hit.seg = kSolid;  // poste, borde o fuera
      break;
    }
    hit.seg = vert ? Maze::vSeg(line, row) : Maze::hSeg(row, line);
    // Dentro de la banda: si llega al poste del extremo antes de salir, choca con su cara.
    const float vOut = base + tOut * va;
    float edge = -1;
    if (vOut > (row + 1) * c - hw_) edge = (row + 1) * c - hw_;
    else if (vOut < row * c + hw_) edge = row * c + hw_;
    if (edge >= 0 && r.n < kMaxHits && fabsf(va) > 1e-6f) {
      Hit& post = r.hits[r.n++];
      const float tp = tIn + (edge - vIn) / va;
      post.t = tp > 0 ? tp : 0;
      post.seg = kSolid;
      post.cosInc = fabsf(va);
      break;
    }
    if (vert) {
      ix += ux > 0 ? 1 : -1;
      tx += dtx;
    } else {
      iy += uy > 0 ? 1 : -1;
      ty += dty;
    }
  }
}

void Sonda::trace(const Maze& m, float x, float y, float ang, Scan& out) const {
  // Direcciones de los rayos girando el eje con las rotaciones precalculadas (sin trigonometria por rayo).
  const float c = cosf(ang), s = sinf(ang);
  for (int j = 0; j < kRays; ++j) {
    Ray& r = out.rays[j];
    traceRay(m, x, y, c * rayCos_[j] - s * raySin_[j], s * rayCos_[j] + c * raySin_[j], r);
    r.w0 = rayW_[j];
  }
}

float Sonda::predict(const Maze& m, const Scan& s, const Override* ov) const {
  float sw = 0, swd = 0;
  for (int j = 0; j < kRays; ++j) {
    const Ray& r = s.rays[j];
    for (int k = 0; k < r.n; ++k) {
      const Hit& hit = r.hits[k];
      bool closed = hit.seg == kSolid;
      if (!closed) {
        if (!ov || !ov->find(hit.seg, closed)) closed = m.segVotes(hit.seg) > 0;
      }
      if (!closed) continue;
      const float d = hit.t > 10.0f ? hit.t : 10.0f;
      const float w = r.w0 * hit.cosInc / (d * d);
      sw += w;
      swd += w * hit.t;
      break;
    }
  }
  if (sw < wRef_ || sw <= 0) return -1.0f;
  const float d = swd / sw;
  return d > static_cast<float>(maxMm_) ? -1.0f : d;
}

void Sonda::prepare(const Maze& m, const Scan& s, const int* cand, int nc, Combo& out) const {
  for (int j = 0; j < kRays; ++j) {
    const Ray& r = s.rays[j];
    Combo::R& o = out.rays[j];
    o.n = 0;
    for (int k = 0; k < r.n; ++k) {
      const Hit& h = r.hits[k];
      int ci = -1;
      if (h.seg != kSolid) {
        for (int q = 0; q < nc; ++q)
          if (cand[q] == h.seg) ci = q;
        if (ci < 0 && m.segVotes(h.seg) <= 0) continue;  // abierta en el mapa: el rayo pasa
      }
      const float d = h.t > 10.0f ? h.t : 10.0f;
      o.w[o.n] = r.w0 * h.cosInc / (d * d);
      o.wt[o.n] = o.w[o.n] * h.t;
      o.cand[o.n] = static_cast<int8_t>(ci);
      ++o.n;
      if (ci < 0) break;  // siempre cerrada: lo de detras no se ve
    }
  }
}

float Sonda::predictMask(const Combo& c, int mask) const {
  float sw = 0, swd = 0;
  for (int j = 0; j < kRays; ++j) {
    const Combo::R& o = c.rays[j];
    for (int k = 0; k < o.n; ++k) {
      if (o.cand[k] >= 0 && !((mask >> o.cand[k]) & 1)) continue;
      sw += o.w[k];
      swd += o.wt[k];
      break;
    }
  }
  if (sw < wRef_ || sw <= 0) return -1.0f;
  const float d = swd / sw;
  return d > static_cast<float>(maxMm_) ? -1.0f : d;
}

float Sonda::parallelFactor(float theta) const {
  // Pared infinita a distancia perpendicular L: cada rayo lee L / sin(th) con peso sin^3(th) / L^2.
  float a = 0, b = 0;
  for (int j = 0; j < kRays; ++j) {
    const float f = -1.0f + (2.0f * j + 1.0f) / kRays;
    const float w0 = sqrtf(1.0f - f * f);
    float sn = sinf(theta + f * halfFov_);
    if (sn < 0.05f) sn = 0.05f;
    a += w0 * sn * sn;
    b += w0 * sn * sn * sn;
  }
  return a / b;
}

}  // namespace raton
