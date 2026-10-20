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
  for (int j = 0; j < kRays; ++j) rayE_[j] = halfFov_ * rayW_[j];  // media altura del cono en esa columna
}

void Sonda::setVertical(float sensorHeightMm, float tiltDeg, float wallHeightMm, float floorReflect) {
  vertical_ = true;
  h_ = sensorHeightMm;
  for (int i = 0; i < 3; ++i) setTilt(i, tiltDeg);
  wallH_ = wallHeightMm;
  floorR_ = floorReflect;
}

void Sonda::column(int j, float tilt, float t, float cosInc, bool any, float& w, float& wt, float& wf,
                   float& wtf) const {
  w = wt = wf = wtf = 0;
  const float w0 = rayW_[j];
  if (!vertical_) {
    if (!any) return;
    const float d = t > 10.0f ? t : 10.0f;
    w = w0 * cosInc / (d * d);
    wt = w * t;
    return;
  }
  // Angulos (rad) de la columna: de lo a hi, repartidos por igual. Se integra en el angulo vertical e (no
  // basta el angulo medio: la luz del piso crece como sin^3 y la de cerca pesa mucho mas).
  const float lo = tilt - rayE_[j], hi = tilt + rayE_[j], k = w0 / (hi - lo);
  // Por debajo de eFloor el rayo toca el piso antes que la pared; por encima de eTop pasa por encima.
  const float eFloor = any ? -atanf(h_ / (t > 1.0f ? t : 1.0f)) : 0.0f;
  const float floorTop = eFloor < hi ? eFloor : hi;
  if (floorR_ > 0 && floorTop > lo) {
    // Piso a distancia h / sin(s) (s = -e) visto con incidencia sin(s): peso sin^3(s) / h^2.
    const float s1 = -floorTop, s2 = -lo, c1 = cosf(s1), c2 = cosf(s2);
    const float i3 = (c1 - c1 * c1 * c1 / 3.0f) - (c2 - c2 * c2 * c2 / 3.0f);  // integral de sin^3
    const float i2 = 0.5f * (s2 - s1) - 0.25f * (sinf(2.0f * s2) - sinf(2.0f * s1));  // de sin^2
    const float kf = k * floorR_;
    wf = kf * i3 / (h_ * h_);
    wtf = kf * i2 / h_;
    w += wf;
    wt += wtf;
  }
  if (!any) return;
  const float eTop = atanf((wallH_ - h_) / (t > 1.0f ? t : 1.0f));
  const float a = lo > eFloor ? lo : eFloor, b = hi < eTop ? hi : eTop;
  if (b > a) {
    // Pared a t / cos(e) con incidencia cosInc cos(e): peso cosInc cos^3(e) / t^2.
    const float d = t > 10.0f ? t : 10.0f, sa = sinf(a), sb = sinf(b);
    const float i3 = (sb - sb * sb * sb / 3.0f) - (sa - sa * sa * sa / 3.0f);  // integral de cos^3
    const float i2 = 0.5f * (b - a) + 0.25f * (sinf(2.0f * b) - sinf(2.0f * a));  // de cos^2
    const float kw = k * cosInc / (d * d);
    w += kw * i3;
    wt += kw * i2 * d;
  }
}

float Sonda::floorStartMm(int sensor) const {
  const float lo = tilt_[sensor] - halfFov_;
  return vertical_ && lo < 0 ? h_ / tanf(-lo) : 0.0f;
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

void Sonda::trace(const Maze& m, float x, float y, float ang, Scan& out, int sensor) const {
  out.sensor = sensor;
  // Direcciones de los rayos girando el eje con las rotaciones precalculadas (sin trigonometria por rayo).
  const float c = cosf(ang), s = sinf(ang);
  for (int j = 0; j < kRays; ++j) {
    Ray& r = out.rays[j];
    traceRay(m, x, y, c * rayCos_[j] - s * raySin_[j], s * rayCos_[j] + c * raySin_[j], r);
    r.j = j;
  }
}

float Sonda::floorDoubt(float d, float sw, float swf, float swfd) {
  // Si el piso pesa una fraccion fs de la lectura, un error en su reflejo (~50 %) mueve la lectura ~fs x
  // (d - distancia media del piso) / 2, y uno en su distancia (inclinacion, altura) ~fs x 10 %.
  if (swf <= 0 || sw <= 0) return 0;
  const float fs = swf / sw, df = swfd / swf;
  return fs * (0.5f * fabsf(df - d) + 0.1f * df);
}

float Sonda::predict(const Maze& m, const Scan& s, const Override* ov) const {
  float fd;
  return predict(m, s, ov, fd);
}

float Sonda::predict(const Maze& m, const Scan& s, const Override* ov, float& floorSigma) const {
  const float tilt = tilt_[s.sensor];
  float sw = 0, swd = 0, swf = 0, swfd = 0;
  floorSigma = 0;
  for (int j = 0; j < kRays; ++j) {
    const Ray& r = s.rays[j];
    float w = 0, wt = 0, wf = 0, wtf = 0;
    bool found = false;
    for (int k = 0; k < r.n && !found; ++k) {
      const Hit& hit = r.hits[k];
      bool closed = hit.seg == kSolid;
      if (!closed) {
        if (!ov || !ov->find(hit.seg, closed)) closed = m.segVotes(hit.seg) > 0;
      }
      if (!closed) continue;
      column(r.j, tilt, hit.t, hit.cosInc, true, w, wt, wf, wtf);
      found = true;
    }
    if (!found) column(r.j, tilt, 0, 0, false, w, wt, wf, wtf);
    sw += w;
    swd += wt;
    swf += wf;
    swfd += wtf;
  }
  if (sw < wRef_ || sw <= 0) return -1.0f;
  const float d = swd / sw;
  if (d > static_cast<float>(maxMm_)) return -1.0f;
  floorSigma = floorDoubt(d, sw, swf, swfd);
  return d;
}

void Sonda::prepare(const Maze& m, const Scan& s, const int* cand, int nc, Combo& out) const {
  const float tilt = tilt_[s.sensor];
  for (int j = 0; j < kRays; ++j) {
    const Ray& r = s.rays[j];
    Combo::R& o = out.rays[j];
    o.n = 0;
    bool blocked = false;
    for (int k = 0; k < r.n; ++k) {
      const Hit& h = r.hits[k];
      int ci = -1;
      if (h.seg != kSolid) {
        for (int q = 0; q < nc; ++q)
          if (cand[q] == h.seg) ci = q;
        if (ci < 0 && m.segVotes(h.seg) <= 0) continue;  // abierta en el mapa: el rayo pasa
      }
      column(r.j, tilt, h.t, h.cosInc, true, o.w[o.n], o.wt[o.n], o.wf[o.n], o.wtf[o.n]);
      o.cand[o.n] = static_cast<int8_t>(ci);
      ++o.n;
      if (ci < 0) {
        blocked = true;  // siempre cerrada: lo de detras no se ve
        break;
      }
    }
    o.wEnd = o.wtEnd = 0;
    if (!blocked) {
      float a, b;
      column(r.j, tilt, 0, 0, false, o.wEnd, o.wtEnd, a, b);
    }
  }
}

float Sonda::predictMask(const Combo& c, int mask, float* floorSigma) const {
  float sw = 0, swd = 0, swf = 0, swfd = 0;
  for (int j = 0; j < kRays; ++j) {
    const Combo::R& o = c.rays[j];
    int k = 0;
    for (; k < o.n; ++k) {
      if (o.cand[k] >= 0 && !((mask >> o.cand[k]) & 1)) continue;
      sw += o.w[k];
      swd += o.wt[k];
      swf += o.wf[k];
      swfd += o.wtf[k];
      break;
    }
    if (k == o.n) {
      sw += o.wEnd;
      swd += o.wtEnd;
      swf += o.wEnd;
      swfd += o.wtEnd;
    }
  }
  if (floorSigma) *floorSigma = 0;
  if (sw < wRef_ || sw <= 0) return -1.0f;
  const float d = swd / sw;
  if (d > static_cast<float>(maxMm_)) return -1.0f;
  if (floorSigma) *floorSigma = floorDoubt(d, sw, swf, swfd);
  return d;
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
