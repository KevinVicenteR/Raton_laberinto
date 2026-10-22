#include "raton/mouse.h"
#include <math.h>
#include <stdio.h>

namespace raton {
namespace {
struct Reference { float mm[5] = {}; };
bool reference(Navigator& nav, Platform& io, Reference& out) {
  int n[5] = {}; float sum[5] = {}, sq[5] = {};
  const float start = nav.now();
  while (nav.now() - start < 0.7f) {
    nav.step();
    if (io.stopRequested() || nav.sensorFault()) return false;
    const auto& s = nav.sensors();
    const Range* rr[5] = {&s.left, &s.front, &s.right, &s.sideLeft, &s.sideRight};
    for (int i=0;i<5;++i) if (rr[i]->fresh && rr[i]->mm >= 10) {
      ++n[i]; sum[i] += rr[i]->mm; sq[i] += float(rr[i]->mm)*rr[i]->mm;
    }
  }
  for (int i=0;i<5;++i) {
    if (n[i] < 8) return false;
    out.mm[i] = sum[i]/n[i];
    if (sq[i]/n[i] - out.mm[i]*out.mm[i] > 36) return false;
  }
  return true;
}
}

bool Mouse::calibrateAll(Params& out) {
  const bool persistent = nav_.persistent();
  nav_.setAutonomous(true);
  // Every exit removes drive power. A failure is never labelled calibrated/home.
  struct Finish {
    Platform& io; Navigator& nav; bool previous;
    ~Finish() { io.setMotors(0,0); nav.setAutonomous(previous); }
  } finish{io_,nav_,persistent};
  const Dir start = static_cast<Dir>(out.startDir & 3);
  nav_.begin(out.startX,out.startY,start);
  io_.report("CAL_COMPLETA phase=REFERENCIAS",true);
  Reference initial;
  if (!reference(nav_,io_,initial)) {
    io_.report("CAL_COMPLETA_ERROR cinco sensores estables y paredes visibles requeridos",true); return false;
  }
  // Known corridor and centred placement are necessary: range bias and placement are not identifiable otherwise.
  const float sideExpected = 0.5f*(out.cellMm-out.wallMm)-out.sideLateralMm;
  if (sideExpected < 15 || fabsf(initial.mm[3]-sideExpected)>25 ||
      fabsf(initial.mm[4]-sideExpected)>25 ||
      0.5f*(out.cellMm-out.wallMm) < bodyEnvelope(out)+6) {
    io_.report("CAL_COMPLETA_ERROR centrar en pasillo conocido con espacio para girar",true); return false;
  }
  io_.report("CAL_COMPLETA phase=PAREDES_Y_GIRO",true);
  if (calibrateSensors(out,true,45) <= 0 || io_.stopRequested() || nav_.sensorFault()) return false;
  out.sideLeftOffsetMm += int(floorf(sideExpected-initial.mm[3]+0.5f));
  out.sideRightOffsetMm += int(floorf(sideExpected-initial.mm[4]+0.5f));
  out.sideEnabled = 1;
  if(abs(out.frontOffsetMm)>60 || abs(out.leftOffsetMm)>60 || abs(out.rightOffsetMm)>60) {
    io_.report("CAL_COMPLETA_ERROR offsets fuera de rango",true);return false;
  }
  if (abs(out.sideLeftOffsetMm)>60 || abs(out.sideRightOffsetMm)>60) return false;
  Reference home;
  if (!reference(nav_,io_,home) || fabsf(nav_.motion().position())>4 ||
      fabsf(nav_.aimAngle())>3) return false;
  const float travel = 0.65f*out.cellMm;
  if (home.mm[1] < travel + out.bodyFrontMm-out.frontAheadMm+30 || home.mm[1]>600) {
    io_.report("CAL_COMPLETA_ERROR falta pared frontal de referencia o espacio para avance",true); return false;
  }
  auto& mo = nav_.motion();
  const float origin=mo.position(), heading=mo.angle();
  const float oldAccel=out.accel;
  out.accel = oldAccel < 500 ? oldAccel : 500;
  float vSum[2]={},uSum[2]={};int count[2]={};
  bool valid=true;
  auto move = [&](float target,float speed,int fit) {
    mo.moveTo(target,speed,0);
    const float t0=nav_.now();float settled=-1;
    while(nav_.now()-t0<6) {
      nav_.step();
      if(io_.stopRequested() || nav_.sensorFault())return false;
      const auto& s=nav_.sensors();
      if (speed>0 && target>mo.position() && s.front.fresh && s.front.mm>=0 &&
          s.front.mm+out.frontAheadMm-out.bodyFrontMm<18) return false;
      if(fabsf(mo.angle()-heading)>5 || fabsf(mo.idealPosition()-mo.position())>20)return false;
      if (fit>=0 && mo.speed()>speed-2 && fabsf(mo.measuredSpeed()-speed)<8 &&
          fabsf(mo.idealPosition()-mo.position())<3) {
        vSum[fit]+=mo.measuredSpeed();uSum[fit]+=0.5f*(mo.dutyLeft()+mo.dutyRight());++count[fit];
      }
      if(mo.arrived() && fabsf(mo.position()-target)<2 && fabsf(mo.measuredSpeed())<5) {
        if(settled<0)settled=nav_.now();
        if(nav_.now()-settled>0.15f)return true;
      } else settled=-1;
    }
    return false;
  };
  io_.report("CAL_COMPLETA phase=MOTORES_AVANCE",true);
  valid=move(origin+0.5f*travel,50,0);
  if(valid)valid=move(origin+travel,90,1);
  Reference away;
  const float moved=mo.position()-origin;
  if(valid)valid=reference(nav_,io_,away);
  const float observed=home.mm[1]-away.mm[1];
  if(valid)valid=observed>0.75f*travel && observed<1.25f*travel &&
                  fabsf(observed-moved)<0.15f*travel;
  mo.hold(nav_.sensors());
  io_.setMotors(0,0);
  io_.report("CAL_COMPLETA phase=REGRESO",true);
  // Return only along the short, just-observed clear segment; never back blindly after stop/IMU/encoder failure.
  bool returned=false;
  if(!io_.stopRequested() && !nav_.sensorFault())returned=move(origin,50,-1);
  mo.hold(nav_.sensors());io_.setMotors(0,0);
  Reference back;
  if(returned)returned=reference(nav_,io_,back);
  if(returned) {
    for(int i=0;i<5;++i)if(fabsf(back.mm[i]-home.mm[i])>8)returned=false;
    returned=returned && fabsf(mo.position()-origin)<3 && fabsf(mo.angle()-heading)<2;
  }
  out.accel=oldAccel;
  if(!valid || !returned) {
    io_.report("CAL_COMPLETA_ERROR avance o regreso no verificado; ajustes no guardados",true);return false;
  }
  // Fit the effective motor feedforward only from two observed steady speeds, never guessed PID gains.
  if(count[0]<20 || count[1]<20)return false;
  const float v0=vSum[0]/count[0],v1=vSum[1]/count[1];
  const float u0=uSum[0]/count[0],u1=uSum[1]/count[1];
  const float kv=(u1-u0)/(v1-v0),ks=u0-kv*v0;
  if(v1-v0<20 || !isfinite(kv) || kv<0.0001f || kv>0.01f || ks<0 || ks>0.4f)return false;
  out.kV=kv;out.kS=ks;
  // Range displacement calibrates the effective encoder distance scale (CPR is retained).
  out.wheelDiamMm *= observed/moved;
  out.learnedCellMm=out.learnedCellRefMm=out.learnedWheelDiamMm=out.learnedCountsPerRev=0;
  out.precisionMode=1;
  char log[144];
  snprintf(log,sizeof(log),"CAL_COMPLETA phase=REGRESO_VERIFICADO avance=%d real=%d error_regreso=%d mm",int(moved),int(observed),int(back.mm[1]-home.mm[1]));
  io_.report(log,true);
  return true;
}
} // namespace raton
