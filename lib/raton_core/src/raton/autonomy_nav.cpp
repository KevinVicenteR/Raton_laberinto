#include "raton/navigator.h"
#include <stdio.h>

namespace raton {
bool Navigator::resumeAfterStorage() {
  io_.setMotors(0,0);
  if(sensorFault() || io_.stopRequested())return false;
  const Sensors before=s_;
  int count[3]={},gyro=0;
  s_=io_.tick();t_=s_.t;
  const float limit=t_+0.3f;
  while(t_<limit){
    s_=io_.tick();t_=s_.t;
    if(io_.stopRequested() || !isfinite(s_.gyroDeg))return false;
    const Range* rr[3]={&s_.left,&s_.front,&s_.right};
    for(int i=0;i<3;++i)if(rr[i]->fresh){++count[i];rangeSeen_[i]=t_;}
    if(s_.gyroFresh)++gyro;
    if(gyro>=3 && s_.gyroFresh && count[1]>=3 && (count[0]>=3 || count[2]>=3)){
      if(fabsf(s_.encLeftMm-before.encLeftMm)>2 || fabsf(s_.encRightMm-before.encRightMm)>2 ||
          fabsf(s_.gyroDeg-before.gyroDeg)>3)return false;
      motion_.hold(s_);
      float l=0,r=0;motion_.update(s_,l,r); // refresh watchdogs on a NEW IMU sample, preserve frame
      motion_.hold(s_);
      return !motion_.sensorFault();
    }
  }
  return false;
}

bool Navigator::safeCurve(int x, int y, Dir incoming, Dir outgoing) const {
  if (!p_.autoGeometryConfirmed || latAge_ > 0.1f || lateral_.varY() > 16 ||
      longVar_ > 25 || fabsf(lateral_.y()) > 4 || fabsf(headingError()) > 2 ||
      rangeLost_[0] || rangeLost_[1] || rangeLost_[2]) return false;
  const int px = x - dx(incoming), py = y - dy(incoming);
  const int nx = x + dx(outgoing), ny = y + dy(outgoing);
  return maze_.fullyKnown(x,y) && maze_.visited(x,y) && maze_.visited(px,py) &&
      maze_.visited(nx,ny) && maze_.fullyKnown(nx,ny) &&
      maze_.votes(px,py,incoming) <= -3 && maze_.votes(x,y,outgoing) <= -3;
}

void Navigator::diagnose() {
  if (t_ - lastHealthReport_ < 0.5f) return;
  lastHealthReport_ = t_;
  char line[190];
  snprintf(line, sizeof(line),
      "HEALTH phase=%s reason=%s age=%d,%d,%d,%d,%d sigma=%d,%d heading=%d repairs=%d",
      sensorFault() ? "PARADO" : recovering_ ? "RECUPERACION" : "EN_MARCHA",
      motion_.encoderFault() ? "ENCODER" : motion_.sensorFault() ? "IMU" : rangeFault_ ? "LASER" :
      motion_.gyroDisagreement() ? "PATINAJE" : "VIGILANDO",
      int(1000*(t_-rangeSeen_[0])), int(1000*(t_-rangeSeen_[1])), int(1000*(t_-rangeSeen_[2])),
      int(1000*(t_-sideSeen_[0])), int(1000*(t_-sideSeen_[1])),
      int(sqrtf(lateral_.varY())), int(sqrtf(longVar_)), int(headingError()), repairAttempts_);
  io_.report(line, false);
  const float pos=cellPos(), lat=lateral_.y();
  const float x=(cx_+0.5f)*cell_+pos*dx(h_)-lat*dy(h_);
  const float y=(cy_+0.5f)*cell_+pos*dy(h_)+lat*dx(h_);
  const int idx=maze_.index(cx_,cy_);
  const bool confirmed=maze_.inside(cx_,cy_) && (confirmedCells_[idx>>3]&(1<<(idx&7)));
  snprintf(line,sizeof(line),"POSE x=%d y=%d sigma=%d,%d visited=%d confirmed=%d collisions=%d oscillations=%d phase=%s",
      int(x),int(y),int(sqrtf(longVar_)),int(sqrtf(lateral_.varY())),maze_.visits(cx_,cy_),confirmed,
      collisionCount_,oscillationCount_,stalled_?"ATASCADO":t_-lastExternalProgress_<0.3f?"AVANCE_CONTRASTADO":"ESTIMADA");
  io_.report(line,false);
}

bool Navigator::repairRanges() {
  io_.setMotors(0,0);
  if (!autonomous_ || motion_.sensorFault() || io_.stopRequested() || repairAttempts_ >= 3) return false;
  // Wait stationary without clearing a latched fault or resetting the pose.
  const float settleStart=t_;
  while(t_-settleStart<0.25f) {
    s_=io_.tick();t_=s_.t;
    if(io_.stopRequested())return false;
  }
  const Sensors before = s_;
  const float start = t_;
  while (t_ - start < 0.12f) {
    s_ = io_.tick(); t_ = s_.t;
    if (io_.stopRequested()) return false;
  }
  if (fabsf(s_.encLeftMm-before.encLeftMm)>2 || fabsf(s_.encRightMm-before.encRightMm)>2 ||
      !isfinite(s_.gyroDeg) || fabsf(s_.gyroDeg-before.gyroDeg)>3) return false;
  const bool wasRequiredFault = rangeFault_;
  if (wasRequiredFault && (fabsf(headingError()) > 3 || fabsf(lateral_.y()) > 8 ||
                          fabsf(cellPos()) > 0.4f * cell_)) return false;
  ++repairAttempts_;
  bool attempted = false, requiredOk = true;
  for (int i = 0; i < 5; ++i) {
    if(i>=3 && !p_.sideEnabled)continue;
    const bool bad = i < 3 ? rangeLost_[i] || health_[i].suspect :
                              health_[i].suspect || t_ - sideSeen_[i-3] > 0.15f;
    if (!bad) continue;
    attempted = true;
    char line[80];
    snprintf(line,sizeof(line),"AUTO phase=REPARACION reason=REINICIAR_XSHUT sensor=%d attempt=%d",i,repairAttempts_);
    io_.report(line,true);
    const bool ok = io_.recoverRange(i);
    if (!ok && (i == 1 || (i < 3 && rangeLost_[0] && rangeLost_[2]))) requiredOk = false;
    if (ok) health_[i] = RangeWatch();
    if (io_.stopRequested()) return false;
  }
  if (!attempted || !requiredOk) return false;
  int fresh[5] = {}, gyro = 0;
  const float until = t_ + 0.35f;
  while (t_ < until) {
    s_ = io_.tick(); t_ = s_.t;
    if (io_.stopRequested()) return false;
    const Range* rr[5] = {&s_.left,&s_.front,&s_.right,&s_.sideLeft,&s_.sideRight};
    for (int i=0;i<5;++i) if (rr[i]->fresh && !health_[i].suspect) {
      ++fresh[i];
      if (i<3) rangeSeen_[i]=t_; else sideSeen_[i-3]=t_;
    }
    if (s_.gyroFresh && isfinite(s_.gyroDeg)) ++gyro;
  }
  // Reappearance alone is insufficient: require several samples and no motion.
  if (fresh[1]<3 || (fresh[0]<3 && fresh[2]<3) || gyro<3 ||
      fabsf(s_.encLeftMm-before.encLeftMm)>2 || fabsf(s_.encRightMm-before.encRightMm)>2 ||
      !isfinite(s_.gyroDeg) || fabsf(s_.gyroDeg-before.gyroDeg)>3) return false;
  for (int i=0;i<3;++i) rangeLost_[i]=fresh[i]<3;
  rangeFault_=false; faultReported_=false;
  motion_.hold(s_);
  if (wasRequiredFault) {
    recovering_ = true;
    motion_.moveTo(centerOdo_,100,0);
    const bool centered = waitStopped(1.5f);
    recovering_ = false;
    if (!centered || fabsf(cellPos()) > 3) {
      rangeFault_ = true; io_.setMotors(0,0); return false;
    }
  }
  resetBeams();
  lateral_.addHeadingVariance(4e-4f);
  longVar_ += 9;
  io_.report("AUTO phase=REPARADO reason=MUESTRAS_Y_REPOSO_VALIDOS",true);
  return true;
}

int Navigator::continuousRecenter(float y, Brain& brain) {
  // Only a measured chassis and a completely known straight corridor allow this
  // maneuver. Otherwise the existing bounded stepwise rescue remains available.
  if (!p_.autoGeometryConfirmed || !maze_.fullyKnown(cx_,cy_) ||
      maze_.wall(cx_,cy_,turnLeft(h_)) != Wall::Closed ||
      maze_.wall(cx_,cy_,turnRight(h_)) != Wall::Closed || fabsf(y)>22) return false;
  const float length = fabsf(y) * 4.0f;
  const float forwardRoom = 0.5f*(cell_-p_.wallMm)-p_.bodyFrontMm-12-cellPos();
  if (forwardRoom < length || halfCorr_-fabsf(y) < 0.5f*p_.bodyWidthMm+8) return false;
  io_.report("AUTO phase=RECENTRADO reason=CURVA_CONTINUA_EN_ESPACIO_CONOCIDO",true);
  const float start=motion_.position(), until=t_+2.5f;
  float estimatedY=y;
  motion_.moveTo(start+length,100,0);
  float last=start;
  while(t_<until && !motion_.arrived()) {
    tick();
    if(stopped() || brain.shouldAbort() || stalled_) { motion_.setSteer(0); return -1; }
    estimatedY += (motion_.position()-last)*sinf(headingError()*0.01745329252f);
    last=motion_.position();
    const float fraction=(last-start)/length;
    const float shape=sinf(3.14159265f*(fraction<1?fraction:1));
    motion_.setSteer((y>0?-1:1)*30.0f*shape*shape);
  }
  motion_.setSteer(0);
  motion_.turnTo(gridAngle_,180,p_.rotAccel);
  if(!waitStopped(0.8f) || brain.shouldAbort()) return -1;
  lateral_.setY(estimatedY,36);
  return fabsf(estimatedY)<8 ? 1 : -1;
}

bool Navigator::diagonal(Dir next, Brain& brain) {
  // Stop/45/straight/45/stop: conservative experimental diagonal. Mapping and
  // cardinal pose integration are suspended until the physical endpoint is reached.
  const Dir first=h_;
  const int ex=cx_+dx(first)+dx(next), ey=cy_+dy(first)+dy(next);
  const int q=next==turnLeft(first)?1:-1;
  const float startAngle=gridAngle_;
  trajectory_=true;
  struct Guard { bool& b; ~Guard(){b=false;} } guard{trajectory_};
  io_.report("AUTO phase=DIAGONAL reason=ENVOLVENTE_Y_POSTES_LIBRES",true);
  auto wait=[&](float limit) {
    const float begin=t_;
    while(t_-begin<limit) {
      tick();
      if(stopped() || brain.shouldAbort() || stalled_) return false;
      const float clearance=p_.bodyFrontMm-p_.frontAheadMm+12;
      if(s_.front.fresh && s_.front.mm>=0 && s_.front.mm<clearance) return false;
      if(motion_.arrived() && !motion_.turning() && fabsf(motion_.measuredSpeed())<12 &&
          fabsf(motion_.position()-motion_.endPosition())<3 &&
          fabsf(motion_.angle()-motion_.headingTarget())<2) return true;
    }
    return false;
  };
  motion_.setSteer(0);
  motion_.moveTo(motion_.position()+0.5f*cell_,120,0);
  if(!wait(2)) { io_.setMotors(0,0); return false; }
  motion_.turnTo(startAngle+q*45,180,p_.rotAccel);
  if(!wait(2)) { io_.setMotors(0,0); return false; }
  motion_.moveTo(motion_.position()+cell_*0.70710678f,120,0);
  if(!wait(3)) { io_.setMotors(0,0); return false; }
  motion_.turnTo(startAngle+q*90,180,p_.rotAccel);
  if(!wait(2)) { io_.setMotors(0,0); return false; }
  motion_.moveTo(motion_.position()+0.5f*cell_,120,0);
  if(!wait(2)) { io_.setMotors(0,0); return false; }
  cx_=ex; cy_=ey; h_=next; gridAngle_=startAngle+q*90;
  centerOdo_=lastOdo_=motion_.position();
  lateral_.reset(0,36,7e-4f); latAge_=1e9f; longVar_=36;
  maze_.markVisited(cx_,cy_);
  for(auto& e:sideEvidence_) e=SideEvidence();
  startStraight(); resetBeams();
  decided_=false; act_=Act::None; blocked_=false;
  io_.report("AUTO phase=DIAGONAL_FIN reason=EXTREMO_CONFIRMADO",true);
  return true;
}
}  // namespace raton
