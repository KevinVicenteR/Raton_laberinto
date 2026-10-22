// Post-calibration stationary sensor check. Always takes a NEW baseline: the
// previous Board sample can precede a blocking gyro calibration by 600+ ms.
#pragma once
#include <math.h>
#include "raton/platform.h"

namespace raton {
struct StartupCheck {
  enum class Action { Normal, Cautious, Cancel };
  int samples[5]={};
  float elapsed=0, driftLeft=0, driftRight=0;
  bool cancelled=false, invalidEncoders=false, invalidClock=false;
  Action action() const {
    if(cancelled || invalidEncoders || invalidClock || elapsed<0.25f)return Action::Cancel;
    return failure()?Action::Cautious:Action::Normal;
  }
  const char* failure() const {
    if(cancelled)return "ARRANQUE_CANCELADO";
    if(invalidClock || elapsed<0.25f)return "RELOJ_AUTOPRUEBA";
    if(invalidEncoders)return "ENCODER_NO_FINITO";
    if(driftLeft>2 || driftRight>2)return "ROBOT_EN_MOVIMIENTO";
    if(samples[1]<3)return "FRONTAL_SIN_MUESTRAS";
    if(samples[0]<3 && samples[2]<3)return "DIAGONALES_SIN_MUESTRAS";
    return nullptr;
  }
};

inline StartupCheck checkStartup(Platform& io) {
  StartupCheck result;
  io.setMotors(0,0);
  if(io.stopRequested()){result.cancelled=true;return result;}
  const Sensors baseline=io.tick(); // AFTER gyro calibration, never Board::last()
  if(!isfinite(baseline.t)){result.invalidClock=true;return result;}
  result.invalidEncoders=!isfinite(baseline.encLeftMm)||!isfinite(baseline.encRightMm);
  Sensors s=baseline;
  for(int cycles=0;cycles<2000;++cycles){
    if(io.stopRequested()){result.cancelled=true;return result;}
    if(!isfinite(s.t) || s.t<baseline.t){result.invalidClock=true;return result;}
    result.elapsed=s.t-baseline.t;
    const Range* ranges[5]={&s.left,&s.front,&s.right,&s.sideLeft,&s.sideRight};
    for(int i=0;i<5;++i)if(ranges[i]->fresh)++result.samples[i];
    if(!isfinite(s.encLeftMm)||!isfinite(s.encRightMm))result.invalidEncoders=true;
    else {
      const float l=fabsf(s.encLeftMm-baseline.encLeftMm),r=fabsf(s.encRightMm-baseline.encRightMm);
      if(l>result.driftLeft)result.driftLeft=l;
      if(r>result.driftRight)result.driftRight=r;
    }
    if(result.elapsed>=0.25f)return result;
    s=io.tick();
  }
  result.invalidClock=true;
  return result;
}
} // namespace raton
