// Experimental policies. Fixed memory, no flash writes or hardware dependencies.
#pragma once
#include <math.h>
#include <stdlib.h>
#include "raton/maze.h"
#include "raton/params.h"
#include "raton/platform.h"

namespace raton {
// A constant reading is normal beside a straight wall. Quarantine only when a
// confirmed geometric prediction changes substantially while the value does not.
struct RangeWatch {
  int reference = -1, samples = 0;
  float since = -1, predicted = -1;
  bool suspect = false;
  void observe(const Range& r, float t, float expected, bool certain) {
    if (suspect || !r.fresh) return;
    if (!certain || r.mm < 0 || expected < 0 || !isfinite(expected)) {
      samples = 0; since = -1; return;
    }
    if (since < 0 || abs(r.mm - reference) > 1) {
      reference = r.mm; since = t; predicted = expected; samples = 0;
    }
    ++samples;
    if (samples >= 12 && t - since > 0.6f && fabsf(expected - predicted) > 40 &&
        fabsf(expected - r.mm) > 30) suspect = true;
  }
};

struct OscillationWatch {
  int sign=0, reversals=0;
  float since=-1,lastCross=-1;
  bool observe(float error,float t,bool straight) {
    if(!straight || !isfinite(error)){sign=0;reversals=0;since=-1;return false;}
    if(since<0 || t-since>2.5f){since=t;reversals=0;}
    const int s=error>1.5f?1:error<-1.5f?-1:0;
    if(!s)return false;
    if(sign && s!=sign && t-lastCross>0.12f){++reversals;lastCross=t;}
    sign=s;
    if(reversals>=5){reversals=0;since=t;return true;}
    return false;
  }
};

struct ProgressWatch {
  bool corroborated=false;
  int key=-1,referenceMm=-1;
  float start=-1,odo=0;
  bool observe(int cellKey,const Range& front,float position,float t,bool wallCertain) {
    corroborated=false;
    if(!front.fresh)return false;
    if(!wallCertain || front.mm<0){key=-1;return false;}
    if(key!=cellKey){key=cellKey;referenceMm=front.mm;odo=position;start=t;return false;}
    const float wheel=position-odo;
    corroborated=wheel>15 && referenceMm-front.mm>8 && referenceMm-front.mm<1.7f*wheel;
    if(t-start>0.25f && wheel>45){
      const bool stuck=referenceMm-front.mm<8;
      key=-1;return stuck;
    }
    if(t-start>1.5f)key=-1;
    return false;
  }
};

inline float autonomySpeed(float requested, bool known, float lateralSigma,
                           float heading, int healthy, bool recovering) {
  float limit = requested;
  if (!known && limit > 220) limit = 220;
  if ((lateralSigma > 8 || fabsf(heading) > 4 || healthy < 3) && limit > 150) limit = 150;
  if (recovering && limit > 120) limit = 120;
  return limit;
}

inline float bodyEnvelope(const Params& p) {
  const float length = p.bodyFrontMm > p.bodyRearMm ? p.bodyFrontMm : p.bodyRearMm;
  const float rectangle = sqrtf(length * length + 0.25f * p.bodyWidthMm * p.bodyWidthMm);
  return rectangle > p.bodyDiamMm * 0.5f ? rectangle : p.bodyDiamMm * 0.5f;
}

// A 45-degree segment joins incoming/outgoing edge midpoints of a known
// four-cell square. Include posts even when all surrounding edges are open.
// Unknown edges are obstacles. Circumscribed chassis circle covers all headings.
inline bool diagonalClear(const Maze& m, const Params& p, float pitch,
                          int x, int y, Dir first, Dir second, float uncertainty) {
  if (!p.autoGeometryConfirmed || !isfinite(pitch) || pitch < 100 ||
      !isfinite(uncertainty) || uncertainty > 12 || uncertainty < 0 ||
      first == second || opposite(first) == second) return false;
  const int ax = x + dx(first), ay = y + dy(first);
  const int bx = x + dx(second), by = y + dy(second);
  const int ex = ax + dx(second), ey = ay + dy(second);
  const int xs[4] = {x, ax, bx, ex}, ys[4] = {y, ay, by, ey};
  for (int i = 0; i < 4; ++i)
    if (!m.inside(xs[i], ys[i]) || !m.visited(xs[i], ys[i]) || !m.fullyKnown(xs[i], ys[i])) return false;
  if (!m.passable(x,y,first,false) || !m.passable(x,y,second,false) ||
      !m.passable(ax,ay,second,false) || !m.passable(bx,by,first,false)) return false;
  const float need = bodyEnvelope(p) + 8 + 2 * uncertainty;
  const float sx = (x + 0.5f + 0.5f * dx(first)) * pitch;
  const float sy = (y + 0.5f + 0.5f * dy(first)) * pitch;
  const float tx = (ax + 0.5f + 0.5f * dx(second)) * pitch;
  const float ty = (ay + 0.5f + 0.5f * dy(second)) * pitch;
  for (int i = 0; i <= 40; ++i) {
    const float px = sx + (tx - sx) * i / 40, py = sy + (ty - sy) * i / 40;
    // Sampling margin bounds the unsampled part of the segment.
    const float radius = need + pitch / 40;
    for (int vy = 0; vy <= m.height(); ++vy)
      for (int vx = 0; vx <= m.width(); ++vx) {
        const float a = fabsf(px - vx * pitch) - p.wallMm * 0.5f;
        const float b = fabsf(py - vy * pitch) - p.wallMm * 0.5f;
        const float aa = a > 0 ? a : 0, bb = b > 0 ? b : 0;
        if (aa * aa + bb * bb < radius * radius) return false;
      }
    for (int cy = 0; cy < m.height(); ++cy)
      for (int cx = 0; cx < m.width(); ++cx)
        for (int d = 0; d < 4; ++d) {
          if (m.wall(cx, cy, static_cast<Dir>(d)) == Wall::Open) continue;
          float a, b;
          if (d == NORTH || d == SOUTH) {
            a = fabsf(py - (cy + (d == NORTH ? 1 : 0)) * pitch) - p.wallMm * 0.5f;
            b = fabsf(px - (cx + 0.5f) * pitch) - 0.5f * pitch;
          } else {
            a = fabsf(px - (cx + (d == EAST ? 1 : 0)) * pitch) - p.wallMm * 0.5f;
            b = fabsf(py - (cy + 0.5f) * pitch) - 0.5f * pitch;
          }
          a = a > 0 ? a : 0; b = b > 0 ? b : 0;
          if (a*a + b*b < radius*radius) return false;
        }
  }
  return true;
}
}  // namespace raton
