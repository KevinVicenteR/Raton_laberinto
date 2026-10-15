#include "risk_predictor.h"
#include <algorithm>
#include <limits>

namespace raton {
void riskFeatures(const RiskFrame h[3],float speed,float yaw,float heading,float pending,int action,float out[43]) {
  int k=0;
  for(int t=0;t<3;++t) for(int i=0;i<3;++i) {
    const auto& s=h[t].sensor[i];
    out[k++]=s.valid?std::max(0.f,std::min(300.f,s.mm))/300.f:0;
    out[k++]=s.valid?1.f:0.f;
    out[k++]=std::max(0.f,std::min(1.f,s.confidence));
    out[k++]=std::max(0.f,std::min(1000.f,s.ageMs))/1000.f;
  }
  out[k++]=speed/300; out[k++]=yaw/180; out[k++]=heading/90; out[k++]=pending/155;
  for(int i=0;i<3;++i) out[k++]=action==i?1.f:0.f;
}
float predictRisk(const RiskModel& m,const float x[43]) {
  const float invalid = std::numeric_limits<float>::quiet_NaN();
  for (int i = 0; i < 43; ++i) if (!std::isfinite(x[i])) return invalid;
  float a[32], b[16];
  for (int i = 0; i < 32; ++i) {
    float v = m.b1[i];
    for (int j = 0; j < 43; ++j) v += m.w1[i][j] * x[j];
    if (!std::isfinite(v)) return invalid;
    a[i] = std::max(0.f, v);
  }
  for (int i = 0; i < 16; ++i) {
    float v = m.b2[i];
    for (int j = 0; j < 32; ++j) v += m.w2[i][j] * a[j];
    if (!std::isfinite(v)) return invalid;
    b[i] = std::max(0.f, v);
  }
  float z = m.b3;
  for (int i = 0; i < 16; ++i) z += m.w3[i] * b[i];
  if (!std::isfinite(z)) return invalid;
  // Stable sigmoid for either sign of the logit.
  return z >= 0 ? 1.f / (1.f + std::exp(-z)) : std::exp(z) / (1.f + std::exp(z));
}
}  // namespace raton
