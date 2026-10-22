#pragma once
#include "raton/params.h"
namespace raton {
// Conservative initial profile, not a plant identification or a claim of optimal PID.
inline void applyStableProfile(Params& p) {
  auto cap=[](float& value,float maximum){if(value>maximum)value=maximum;};
  cap(p.kpFwd,0.009f);cap(p.kiFwd,0.006f);
  cap(p.kpRot,0.015f);cap(p.kiRot,0.006f);
  cap(p.kSteer,0.12f);cap(p.steerMaxDeg,2.5f);
  cap(p.accel,900);cap(p.rotSpeed,240);cap(p.rotAccel,1800);
}
}
