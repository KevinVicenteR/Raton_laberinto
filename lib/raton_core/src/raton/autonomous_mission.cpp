#include "raton/mouse.h"
#include "raton/planner.h"
#include <stdio.h>
#include <limits>

namespace raton {
MissionResult Mouse::autonomous() {
  MissionResult result;
  nav_.setAutonomous(true);
  explorer_.setAutonomous(true);
  struct Guard {
    Navigator& nav; Explorer& explorer; Platform& io;
    bool& complete;
    ~Guard() {
      io.setMotors(0,0);
      if(!complete)io.report("AUTO phase=PARADO reason=MISION_NO_COMPLETADA",true);
      nav.snapshot();nav.setAutonomous(false); explorer.setAutonomous(false);
    }
  } guard{nav_, explorer_, io_,result.autonomousComplete};
  // Work in RAM. The persisted map is replaced only after complete mapping and
  // a verified physical return; a failed mission cannot erase it.
  resetMap();
  io_.report("MAPA_NUEVO",true);
  io_.report("INICIO_MAPA",true);
  nav_.begin(p_.startX,p_.startY,static_cast<Dir>(p_.startDir&3));
  // Persistent operation ends on completion, explicit stop or unusable hardware.
  const float start=nav_.now(), deadline=std::numeric_limits<float>::infinity();
  explorer_.setHardDeadline(deadline);
  auto stage=[&](Explorer::Phase phase,const char* name) {
    char line[96];
    snprintf(line,sizeof(line),"AUTO phase=%s reason=TRANSICION_AUTOMATICA",name);
    io_.report(line,true);
    explorer_.setPhase(phase);
    NavResult nr=nav_.run(explorer_,p_.searchSpeed,true);
    // Communication recovery is stationary, bounded and retains the current pose.
    for(int attempt=0;attempt<3 && nr==NavResult::Failed && nav_.sensorFault() &&
        nav_.now()<deadline && !io_.stopRequested();++attempt) {
      if(!nav_.repairRanges()) break;
      nr=nav_.run(explorer_,p_.searchSpeed,false);
    }
    return nr==NavResult::Arrived && !nav_.sensorFault() && !io_.stopRequested() && nav_.now()<deadline;
  };
  io_.report("AUTO phase=DIAGNOSTICO reason=VIGILANCIA_CONTINUA",true);
  if(!stage(Explorer::Phase::ToGoal,"BUSCAR_META")) return finish(result,start);
  result.reachedGoal=maze_.isGoal(nav_.x(),nav_.y());
  if(result.reachedGoal) {
    result.goalTime=nav_.now()-start;
    io_.report("AUTO phase=META_ENCONTRADA reason=CONTINUAR_MAPEO",true);
  }
  explorer_.setFullMapping(true);
  const bool mapped=stage(Explorer::Phase::ExploreAll,"MAPEAR_TODO");
  explorer_.setFullMapping(false);
  if(!mapped) return finish(result,start);
  result.exploredAll=explorer_.exploredAll();
  if(!stage(Explorer::Phase::ToStart,"REGRESAR")) return finish(result,start);
  nav_.face(static_cast<Dir>(p_.startDir&3));
  result.backHome=nav_.x()==p_.startX && nav_.y()==p_.startY &&
      !nav_.sensorFault() && !io_.stopRequested() &&
      fabsf(nav_.aimAngle())<3 && nav_.now()<deadline;
  if(!result.exploredAll || !result.backHome) return finish(result,start);
  io_.setMotors(0,0);
  if(!io_.saveMap(maze_)) {
    io_.report("AUTO phase=PARADO reason=ERROR_GUARDAR_MAPA",true);
    return finish(result,start);
  }
  io_.report("MAPA_GUARDADO",true);
  if(!nav_.resumeAfterStorage()){
    io_.report("AUTO phase=PARADO reason=REFERENCIAS_TRAS_GUARDADO",true);
    return finish(result,start);
  }
  class KnownBrain : public Brain {
   public:
    KnownBrain(Maze& m,Navigator& nav,float end):m_(m),nav_(nav),end_(end){}
    bool shouldAbort() override { return nav_.now()>=end_; }
    int decide(int x,int y,Dir h) override {
      if(m_.isGoal(x,y)) return -1;
      CellSet targets; goalSet(m_,targets);
      bestRoute(m_,x,y,h,targets,false,0,route_);
      return route_.ok && route_.length ? route_.steps[0] : -1;
    }
    bool diagonalPair(int x,int y,Dir h,Dir& next) override {
      decide(x,y,h);
      if(!route_.ok || route_.length<2 || route_.steps[0]!=h ||
          route_.steps[1]==h || route_.steps[1]==opposite(h)) return false;
      next=route_.steps[1]; return true;
    }
   private:
    Maze& m_; Navigator& nav_; float end_; Route route_;
  };
  KnownBrain brain(maze_,nav_,deadline);
  io_.report("AUTO phase=CAMINO_MAS_CORTO reason=SOLO_PASOS_CONFIRMADOS",true);
  // No begin(): the robot is already physically home, with its original gyro frame.
  const NavResult nr=nav_.run(brain,p_.runSpeed,true);
  if(nr==NavResult::Arrived && maze_.isGoal(nav_.x(),nav_.y()) && !nav_.sensorFault()) {
    io_.report("META",true);
    io_.report("AUTO phase=COMPLETO reason=MAPA_GUARDADO_Y_META",true);
    result.autonomousComplete=true;
  } else io_.report("AUTO phase=PARADO reason=CARRERA_INTERRUMPIDA",true);
  return finish(result,start);
}
}  // namespace raton
