#include "raton/autonomy.h"
#include "raton/mouse.h"
#include "sim.h"
#include "run_case.h"
#include "raton/event_journal.h"
#include "raton/startup_check.h"
#include "raton/stable_control.h"
#include <stdio.h>
#include <string.h>
using namespace raton;
static int checks=0, failures=0;
#define CHECK(x) do { ++checks; if(!(x)){++failures;printf("FALLO linea %d: %s\n",__LINE__,#x);} } while(0)

static Maze openMaze(int n) {
  Maze m; m.reset(n,n,true);
  for(int y=0;y<n;++y) for(int x=0;x<n;++x) {
    m.markVisited(x,y);
    if(x+1<n)m.set(x,y,EAST,Wall::Open);
    if(y+1<n)m.set(x,y,NORTH,Wall::Open);
  }
  return m;
}

static void policies() {
  OscillationWatch wobble;bool detected=false;
  for(int i=0;i<7;++i)detected=wobble.observe(i%2?3:-3,0.2f*i,true)||detected;
  CHECK(detected);
  OscillationWatch noise;
  for(int i=0;i<30;++i)CHECK(!noise.observe(i%2?0.3f:-0.3f,0.05f*i,true));
  ProgressWatch slipping;
  CHECK(!slipping.observe(1,Range{100,true},0,0,true));
  CHECK(slipping.observe(1,Range{100,true},50,0.5f,true));
  ProgressWatch moving;
  CHECK(!moving.observe(1,Range{150,true},0,0,true));
  CHECK(!moving.observe(1,Range{100,true},50,0.5f,true));CHECK(moving.corroborated);
  EventJournal journal;char out[192];uint32_t id=0;
  for(int i=0;i<40;++i)journal.push("AUTO phase=PRUEBA reason=OFFLINE",i*100);
  CHECK(journal.overwritten()==8);CHECK(journal.next(0,out,sizeof(out),id));CHECK(id==9);
  int recovered=1;
  while(journal.next(id,out,sizeof(out),id))++recovered;
  CHECK(recovered==32);CHECK(id==40);
  RangeWatch stable;
  for(int i=0;i<100;++i) stable.observe(Range{70,true},i*.025f,70,true);
  CHECK(!stable.suspect); // a straight side wall is legitimately constant
  RangeWatch frozen;
  for(int i=0;i<50;++i) frozen.observe(Range{100,true},i*.025f,100-i*2,true);
  CHECK(frozen.suspect);
  RangeWatch unknown;
  for(int i=0;i<50;++i) unknown.observe(Range{100,true},i*.025f,100-i*2,false);
  CHECK(!unknown.suspect);
  CHECK(autonomySpeed(400,false,2,0,3,false)==220);
  CHECK(autonomySpeed(400,true,12,0,3,false)==150);
  CHECK(autonomySpeed(400,true,2,0,3,true)==120);
  Params p; auto m=openMaze(4);
  CHECK(!diagonalClear(m,p,300,1,1,NORTH,EAST,1));
  p.autoGeometryConfirmed=1;
  CHECK(diagonalClear(m,p,300,1,1,NORTH,EAST,1));
  CHECK(!diagonalClear(m,p,155,1,1,NORTH,EAST,1)); // central post/chassis envelope
  CHECK(!diagonalClear(m,p,300,1,1,NORTH,EAST,20));
  m.set(1,2,EAST,Wall::Unknown);
  CHECK(!diagonalClear(m,p,300,1,1,NORTH,EAST,1));
}

static bool event(const sim::SimPlatform& io,const char* text) {
  for(const auto& e:io.events) if(e.text.find(text)!=std::string::npos)return true;
  return false;
}

class ObservedPlatform : public sim::SimPlatform {
 public:
  using sim::SimPlatform::SimPlatform;
  int healthReports=0;
  bool pauseOnSave=false;
  bool saveMap(const Maze& m) override {
    const bool saved=sim::SimPlatform::saveMap(m);
    if(pauseOnSave){const float until=time()+0.2f;while(time()<until)sim::SimPlatform::tick();}
    return saved;
  }
  void report(const char* line,bool important) override {
    if(strncmp(line,"HEALTH ",7)==0)++healthReports;
    sim::SimPlatform::report(line,important);
  }
};

static void mission(bool cross,bool geometry=false,bool cautious=false,bool stable=false) {
  Params p; if(stable)applyStableProfile(p); p.sideEnabled=1;
  p.mazeW=p.mazeH=5; p.startX=2;p.startY=0;p.goalX=4;p.goalY=2;
  if(geometry){p.cellMm=300;p.autoGeometryConfirmed=1;}
  Maze truth; truth.reset(5,5,true);
  for(int y=0;y<5;++y)for(int x=0;x<5;++x)for(int d=0;d<4;++d)truth.set(x,y,Dir(d),Wall::Closed);
  for(int y=0;y<(cross?4:2);++y)truth.set(2,y,NORTH,Wall::Open);
  for(int x=0;x<4;++x)truth.set(x,2,EAST,Wall::Open);
  auto ph=sim::perturbed(p,0,1);ph.sideSensors=true;
  sim::World world(truth,ph);ObservedPlatform io(world,p,ph,1);io.pauseOnSave=true;io.place(2,0,NORTH);
  Mouse mouse(io,p);mouse.navigator().setCautious(cautious);auto r=mouse.autonomous();
  if(!r.autonomousComplete) for(const auto& e:io.events) if(e.text.find("NAV ")!=0)printf("%.2f %s\n",e.t,e.text.c_str());
  CHECK(r.autonomousComplete);CHECK(r.exploredAll);CHECK(r.backHome);CHECK(io.saved);
  CHECK(mouse.maze().visited(0,2)&&mouse.maze().visited(4,2));
  CHECK(io.cellX()==4 && io.cellY()==2);
  CHECK(io.contacts()==0);
  CHECK(event(io,"phase=MAPEAR_TODO"));CHECK(event(io,"phase=REGRESAR"));
  CHECK(event(io,"phase=CAMINO_MAS_CORTO"));CHECK(io.healthReports>5);
  CHECK(r.totalTime<595);
  printf("Autonomo %s celda %.0f: completo=%d t=%.1f contactos=%d\n",cross?"cruz":"T",p.cellMm,r.autonomousComplete,r.totalTime,io.contacts());
}

static void fatalFault() {
  Params p;p.mazeW=p.mazeH=5;p.startX=2;p.startY=0;p.goalX=4;p.goalY=4;
  auto truth=openMaze(5);auto ph=sim::perturbed(p,0,1);ph.gyroFailAt=2;
  sim::World world(truth,ph);sim::SimPlatform io(world,p,ph,1);io.place(2,0,NORTH);
  Mouse mouse(io,p);auto r=mouse.autonomous();
  CHECK(!r.autonomousComplete);CHECK(!io.saved);CHECK(mouse.navigator().sensorFault());
  CHECK(r.totalTime<3);CHECK(!event(io,"phase=REPARADO"));
}

class RepairPlatform : public ObservedPlatform {
 public:
  using ObservedPlatform::ObservedPlatform;
  bool repairable=true, repaired=false, stoppedForRepair=true;
  int attempts=0;
  float left=0,right=0;
  Sensors snapshot;
  void setMotors(float l,float r) override {left=l;right=r;ObservedPlatform::setMotors(l,r);}
  const Sensors& tick() override {
    snapshot=ObservedPlatform::tick();
    if(time()>0.2f && !repaired) {snapshot.front.fresh=false;snapshot.front.mm=-1;}
    return snapshot;
  }
  bool recoverRange(int i) override {
    ++attempts;stoppedForRepair=stoppedForRepair && left==0 && right==0;
    if(i==1 && repairable){repaired=true;return true;}
    return false;
  }
};

static void communicationRepair(bool works) {
  Params p;p.mazeW=p.mazeH=3;p.startX=1;p.startY=0;p.goalX=1;p.goalY=2;
  Maze truth;truth.reset(3,3,true);
  for(int y=0;y<3;++y)for(int x=0;x<3;++x)for(int d=0;d<4;++d)truth.set(x,y,Dir(d),Wall::Closed);
  truth.set(1,0,NORTH,Wall::Open);truth.set(1,1,NORTH,Wall::Open);
  auto ph=sim::perturbed(p,0,1);sim::World world(truth,ph);
  RepairPlatform io(world,p,ph,1);io.repairable=works;io.place(1,0,NORTH);
  Mouse mouse(io,p);const auto r=mouse.autonomous();
  CHECK(io.attempts>=1 && io.attempts<=3);CHECK(io.stoppedForRepair);
  CHECK(r.autonomousComplete==works);CHECK(io.saved==works);
  CHECK(io.contacts()==0);
  if(works)CHECK(event(io,"phase=REPARADO"));else CHECK(r.totalTime<3);
  printf("Reparacion %s: intentos=%d completo=%d t=%.2f\n",works?"recuperable":"permanente",io.attempts,r.autonomousComplete,r.totalTime);
}

static void diagonalMission() {
  Params p;p.cellMm=300;p.mazeW=p.mazeH=4;p.startX=0;p.startY=0;p.goalX=3;p.goalY=3;
  p.autoGeometryConfirmed=1;p.sideEnabled=1;
  auto truth=openMaze(4);auto ph=sim::perturbed(p,0,1);ph.sideSensors=true;
  sim::World world(truth,ph);ObservedPlatform io(world,p,ph,1);io.place(0,0,NORTH);
  Mouse mouse(io,p);const auto r=mouse.autonomous();
  CHECK(r.autonomousComplete);CHECK(io.cellX()==3&&io.cellY()==3);CHECK(io.contacts()==0);
  // Equal-cost exploration routes may use safe pivots instead of a diagonal.
  // Exercise the diagonal explicitly with a completely known open square below.
  printf("Diagonal/curvas abiertas: completo=%d diagonal=%d contactos=%d\n",r.autonomousComplete,event(io,"phase=DIAGONAL "),io.contacts());
  sim::World knownWorld(truth,ph);ObservedPlatform known(knownWorld,p,ph,1);known.place(0,0,NORTH);
  Maze map=truth;map.setGoal(1,1,1,1);
  Navigator nav(known,p,map);nav.setAutonomous(true);nav.begin(0,0,NORTH);
  class SquareBrain : public Brain {
   public:
    int decide(int x,int y,Dir h) override {return x==1 && y==1?-1:int(h);}
    bool diagonalPair(int x,int y,Dir h,Dir& next) override {
      if(x!=0 || y!=0 || h!=NORTH)return false;
      next=EAST;return true;
    }
  } brain;
  CHECK(nav.run(brain,120,false)==NavResult::Arrived);
  CHECK(event(known,"phase=DIAGONAL "));CHECK(known.cellX()==1 && known.cellY()==1);
  CHECK(known.contacts()==0);
}

static float rotationalNoise(bool stable) {
  Params p;p.learnMotor=0;p.kiRot=0;
  Sensors s;s.gyroFresh=true;
  Motion motion(p);motion.setStableControl(stable);motion.reset(s);motion.cruise(180);
  float jerk=0,previous=0;int n=0;
  for(int i=1;i<=1500;++i){
    s.t=i*0.002f;s.encLeftMm=s.encRightMm=180*s.t;
    s.gyroFresh=i%2==0;
    if(s.gyroFresh)s.gyroDeg=0.18f*sinf(2*3.14159265f*75*s.t);
    float l,r;motion.update(s,l,r);const float differential=0.5f*(r-l);
    if(i>250){jerk+=(differential-previous)*(differential-previous);++n;}
    previous=differential;
  }
  return sqrtf(jerk/n);
}

static float forwardNoise(bool stable) {
  Params p;p.learnMotor=0;p.kiFwd=0;
  Sensors s;s.gyroFresh=true;Motion motion(p);motion.setStableControl(stable);motion.reset(s);motion.cruise(180);
  float jerk=0,previous=0;int n=0;
  for(int i=1;i<=1500;++i) {
    s.t=i*0.002f;s.encLeftMm=s.encRightMm=180*s.t+(i%2?0.15f:-0.15f);
    float l,r;motion.update(s,l,r);const float common=0.5f*(r+l);
    if(i>250){jerk+=(common-previous)*(common-previous);++n;}previous=common;
  }
  return sqrtf(jerk/n);
}

static void stableControlRegression() {
  LateralFilter filter;filter.setRobust(true);filter.reset(0,1,1e-5f);
  for(int i=0;i<12;++i)filter.update(i%2?70:-70,1);
  CHECK(fabsf(filter.y())<1);
  for(int i=0;i<5;++i)filter.update(22,1);
  CHECK(fabsf(filter.y()-22)<1);
  const float old=rotationalNoise(false),filtered=rotationalNoise(true);
  CHECK(filtered<old*0.5f);
  printf("Ruido de mando diferencial: previo=%.6f filtrado=%.6f\n",old,filtered);
  const float rawForward=forwardNoise(false),filteredForward=forwardNoise(true);
  CHECK(filteredForward<rawForward*0.5f);
  printf("Ruido de mando avance: previo=%.6f filtrado=%.6f\n",rawForward,filteredForward);
  Params quietParams;Motion quiet(quietParams);quiet.setStableControl(true);Sensors still;still.gyroFresh=true;quiet.reset(still);
  for(int i=1;i<=200;++i) {
    still.t=0.002f*i;still.gyroDeg=0.1f*sinf(still.t*20);
    float l,r;quiet.update(still,l,r);CHECK(l==0 && r==0);
  }
  Params profile;profile.kpFwd=0.005f;applyStableProfile(profile);
  CHECK(profile.kpFwd==0.005f && profile.kiFwd==0.006f && profile.kiRot==0.006f);
  CHECK(profile.kpRot==0.015f && profile.accel==900 && profile.rotSpeed==240);
  Params p;Motion motion(p);Sensors s;s.gyroFresh=true;motion.reset(s);
  motion.setHeadingReference(2.5f);CHECK(motion.headingTarget()==2.5f);
  motion.turnTo(90,180,900);motion.setHeadingReference(3);
  CHECK(motion.headingTarget()==90); // Wall updates cannot rewrite a live pivot.
}

static void independentObstacleGuard() {
  class NearWall : public Platform {
   public:
    Sensors s;float left=0,right=0;bool guarded=false;
    const Sensors& tick() override {
      s.t+=0.002f;s.gyroFresh=true;
      s.front=Range{14,true};s.left=s.right=Range{60,true};
      s.sideLeft=s.sideRight=Range{40,true};return s;
    }
    void setMotors(float l,float r) override {left=l;right=r;}
    void report(const char* text,bool) override {if(strstr(text,"FRENADO_DISTANCIA_REAL"))guarded=true;}
  } io;
  Params p;Mouse mouse(io,p);auto& nav=mouse.navigator();nav.setAutonomous(true);
  nav.begin(p.startX,p.startY,Dir(p.startDir));
  const auto before=mouse.maze().wall(p.startX,p.startY,Dir(p.startDir));
  nav.motion().cruise(100);for(int i=0;i<50;++i)nav.step();
  CHECK(io.guarded);CHECK(io.left==0 && io.right==0);
  CHECK(mouse.maze().wall(p.startX,p.startY,Dir(p.startDir))==before);
}

static void calibrationRegression() {
  Params p;applyStableProfile(p);p.mazeW=p.mazeH=4;p.startX=1;p.startY=0;p.startDir=0;
  Maze truth;truth.reset(4,4,true);
  for(int y=0;y<4;++y)for(int x=0;x<4;++x)for(int d=0;d<4;++d)truth.set(x,y,Dir(d),Wall::Closed);
  truth.set(1,0,NORTH,Wall::Open);truth.set(1,1,NORTH,Wall::Open);
  auto ph=sim::perturbed(p,0,1);ph.sideSensors=true;ph.rangeNoise=0;ph.rangeNoiseMm=0;
  ph.dropout=ph.outlier=ph.gyroNoiseDps=ph.gyroBiasDps=ph.gyroDriftDps=0;
  sim::World world(truth,ph);ObservedPlatform io(world,p,ph,10);io.place(1,0,NORTH);io.timeLimit=45;
  Mouse mouse(io,p);
  const bool calibrated=mouse.calibrateAll(p);
  printf("Calibracion nominal: ok=%d contactos=%d t=%.2f\n",calibrated,io.contacts(),io.time());
  if(!calibrated)for(const auto& e:io.events)printf("CALTRACE %s\n",e.text.c_str());
  CHECK(calibrated);CHECK(io.contacts()==0);CHECK(io.cellX()==1 && io.cellY()==0);
  CHECK(event(io,"CAL_COMPLETA phase=REGRESO_VERIFICADO"));
  auto missing=ph;missing.sideSensors=false;
  sim::World world2(truth,missing);ObservedPlatform broken(world2,p,missing,10);broken.place(1,0,NORTH);
  Mouse mouse2(broken,p);CHECK(!mouse2.calibrateAll(p));CHECK(broken.time()<2);
  CHECK(!event(broken,"CAL_COMPLETA phase=REGRESO_VERIFICADO"));
}

static void noisyCorridor() {
  Params p;applyStableProfile(p);p.mazeW=p.mazeH=7;p.startX=3;p.startY=0;p.goalX=3;p.goalY=5;
  p.sideEnabled=1;
  Maze truth;truth.reset(7,7,true);
  for(int y=0;y<7;++y)for(int x=0;x<7;++x)for(int d=0;d<4;++d)
    truth.set(x,y,Dir(d),Wall::Closed);
  for(int y=0;y<6;++y)truth.set(3,y,NORTH,Wall::Open);
  auto ph=sim::perturbed(p,0,1);ph.sideSensors=true;ph.motorAsym=0.08f;
  ph.startX=6;ph.startDeg=1;ph.rangeNoise=0.04f;ph.gyroNoiseDps=0.25f;
  sim::World world(truth,ph);ObservedPlatform io(world,p,ph,41);io.place(3,0,NORTH);io.timeLimit=60;
  Mouse mouse(io,p);mouse.navigator().setAutonomous(true);io.watch=&mouse.maze();
  const auto r=mouse.toGoal();
  CHECK(r.reachedGoal);CHECK(io.cellX()==3 && io.cellY()==5);
  CHECK(io.contacts()==0);CHECK(io.offsetMax()<25);CHECK(io.wobbleRms()<5);
  printf("Pasillo con ruido/asimetria: meta=%d contactos=%d lateralMax=%.2f rumboRMS=%.2f\n",
         r.reachedGoal,io.contacts(),io.offsetMax(),io.wobbleRms());
}

static int benchmark(int n) {
  int failed=0;
  for(int k=0;k<n;++k){
    Params p;p.sideEnabled=1;
    Maze truth;sim::randomMaze(p.mazeW,p.mazeH,12,1000u+k,truth);
    sim::makeStartCell(truth,p.startX,p.startY,Dir(p.startDir),1000u+k);
    const auto ph=sim::perturbed(p,2,77u+k);
    sim::World world(truth,ph);ObservedPlatform io(world,p,ph,5u+k);io.place(p.startX,p.startY,p.startDir);
    Mouse mouse(io,p);const auto r=mouse.autonomous();
    if(!r.autonomousComplete)++failed;
    printf("AUTO_CASE %d complete=%d goal=%d mapped=%d home=%d contacts=%d time=%.1f\n",1000+k,r.autonomousComplete,r.reachedGoal,r.exploredAll,r.backHome,io.contacts(),r.totalTime);
    fflush(stdout);
  }
  printf("AUTO_BENCH %d/%d completed\n",n-failed,n);return failed?1:0;
}

class StartupPlatform : public Platform {
 public:
  Sensors s;
  float clock=10.6f;
  bool missingFront=false,missingLeft=false,missingRight=false,move=false,returnHome=false,cancel=false;
  bool invalidEncoder=false,noClock=false,invalidClock=false;
  int ticks=0;
  bool movingOutput=false;
  void setMotors(float l,float r) override {movingOutput=movingOutput||l!=0||r!=0;}
  bool stopRequested() override{return cancel && ticks>=10;}
  void report(const char*,bool) override{}
  const Sensors& tick() override {
    ++ticks;if(!noClock)clock+=0.002f;s.t=invalidClock?NAN:clock;
    s.left.fresh=s.front.fresh=s.right.fresh=s.sideLeft.fresh=s.sideRight.fresh=false;
    if(ticks%10==0){
      s.left=Range{-1,!missingLeft};s.front=Range{-1,!missingFront};s.right=Range{-1,!missingRight};
    }
    // Out-of-range samples are fresh transport responses, not disconnections.
    s.encLeftMm=invalidEncoder?NAN:move && (!returnHome || ticks<70)?3:0;
    return s;
  }
};

static void startupRegression(){
  StartupPlatform io;
  io.s.t=10.0f; // sample cached before a 600 ms blocking calibration
  const auto good=checkStartup(io);
  CHECK(good.failure()==nullptr);CHECK(good.elapsed>=0.25f);CHECK(io.ticks>=125);
  CHECK(good.samples[1]>=10);CHECK(!io.movingOutput);
  StartupPlatform front;front.missingFront=true;
  CHECK(strcmp(checkStartup(front).failure(),"FRONTAL_SIN_MUESTRAS")==0);
  StartupPlatform lateral;lateral.missingLeft=true;
  CHECK(checkStartup(lateral).failure()==nullptr);
  lateral.missingRight=true;
  CHECK(strcmp(checkStartup(lateral).failure(),"DIAGONALES_SIN_MUESTRAS")==0);
  // Movement after baseline, then return to it, must still be reported.
  struct Moved : StartupPlatform {
    const Sensors& tick() override {
      StartupPlatform::tick();s.encLeftMm=ticks>20 && ticks<70?3:0;return s;
    }
  } moved;
  CHECK(strcmp(checkStartup(moved).failure(),"ROBOT_EN_MOVIMIENTO")==0);
  StartupPlatform partial;partial.missingFront=true;
  CHECK(checkStartup(partial).action()==StartupCheck::Action::Cautious);
  StartupPlatform invalid;invalid.invalidEncoder=true;
  CHECK(checkStartup(invalid).action()==StartupCheck::Action::Cancel);
  StartupPlatform normal;
  CHECK(checkStartup(normal).action()==StartupCheck::Action::Normal);
  StartupPlatform nan;nan.invalidEncoder=true;
  CHECK(strcmp(checkStartup(nan).failure(),"ENCODER_NO_FINITO")==0);
  StartupPlatform cancelled;cancelled.cancel=true;
  CHECK(checkStartup(cancelled).cancelled);CHECK(!cancelled.movingOutput);
  StartupPlatform stoppedClock;stoppedClock.noClock=true;
  CHECK(checkStartup(stoppedClock).invalidClock);
  StartupPlatform brokenClock;brokenClock.invalidClock=true;
  CHECK(checkStartup(brokenClock).invalidClock);
}

// The obstruction lasts longer than the former three-attempt budget.
class PersistentJam : public ObservedPlatform {
 public:
  using ObservedPlatform::ObservedPlatform;
  float releaseAt=12, lastLeft=0, lastRight=0;
  void setMotors(float l,float r) override {
    lastLeft=l;lastRight=r;
    ObservedPlatform::setMotors(time()<releaseAt?0:l,time()<releaseAt?0:r);
  }
};

static void persistentRecovery(int mode,bool locked=false) {
  Params p;p.mazeW=p.mazeH=3;p.startX=1;p.startY=0;p.goalX=1;p.goalY=2;
  Maze truth;truth.reset(3,3,true);
  for(int y=0;y<3;++y)for(int x=0;x<3;++x)for(int d=0;d<4;++d)
    truth.set(x,y,Dir(d),Wall::Closed);
  truth.set(1,0,NORTH,Wall::Open);truth.set(1,1,NORTH,Wall::Open);
  auto ph=sim::perturbed(p,0,1);sim::World world(truth,ph);
  PersistentJam io(world,p,ph,1);io.place(1,0,NORTH);
  io.releaseAt=locked?1000:12;
  io.timeLimit=locked?610:90;
  Mouse mouse(io,p);mouse.navigator().setAutonomous(true);
  MissionResult r;
  if(mode==1)r=mouse.autonomous();
  else if(mode==2)r=mouse.toGoal();
  else {mouse.maze()=truth;r=mouse.solveLearned();}
  CHECK(r.recoveries>3);CHECK(!mouse.navigator().sensorFault());
  CHECK(!event(io,"RECUPERACION_AGOTADA"));
  CHECK(event(io,"reason=REINTENTO_PERSISTENTE"));
  CHECK(io.lastLeft==0 && io.lastRight==0);
  CHECK(io.contacts()==0);
  // A mechanical obstruction must not become a fictitious permanent wall.
  CHECK(mouse.maze().wall(1,0,NORTH)!=Wall::Closed);
  if(locked){
    CHECK(!r.reachedGoal);CHECK(io.time()>io.timeLimit);CHECK(r.totalTime>609.9f && r.totalTime<613);
    CHECK(!io.saved);CHECK(io.cellX()==1 && io.cellY()==0);
  } else {
    CHECK(r.reachedGoal);CHECK(io.cellX()==1 && io.cellY()==2);
    if(mode==1)CHECK(r.autonomousComplete);
    CHECK(r.totalTime>12 && r.totalTime<90);
  }
  printf("Atasco persistente modo=%d bloqueo=%d recuperaciones=%d t=%.2f meta=%d\n",
         mode,locked,r.recoveries,r.totalTime,r.reachedGoal);
}

int main(int argc,char** argv) {
  if(argc>1 && !strcmp(argv[1],"--bench"))return benchmark(argc>2?atoi(argv[2]):8);
  policies();mission(false,false,false,true);mission(false);mission(true);mission(true,true);fatalFault();
  mission(false,false,true);
  startupRegression();
  stableControlRegression();
  noisyCorridor();
  calibrationRegression();
  independentObstacleGuard();
  persistentRecovery(1);persistentRecovery(2);persistentRecovery(3);
  persistentRecovery(2,true);
  communicationRepair(true);communicationRepair(false);diagonalMission();
  printf("Autonomia: %d comprobaciones, %d fallos\n",checks,failures);
  return failures?1:0;
}
