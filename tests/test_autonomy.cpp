#include "raton/autonomy.h"
#include "raton/mouse.h"
#include "sim.h"
#include "run_case.h"
#include "raton/event_journal.h"
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

static void mission(bool cross,bool geometry=false) {
  Params p; p.sideEnabled=1;
  p.mazeW=p.mazeH=5; p.startX=2;p.startY=0;p.goalX=4;p.goalY=2;
  if(geometry){p.cellMm=300;p.autoGeometryConfirmed=1;}
  Maze truth; truth.reset(5,5,true);
  for(int y=0;y<5;++y)for(int x=0;x<5;++x)for(int d=0;d<4;++d)truth.set(x,y,Dir(d),Wall::Closed);
  for(int y=0;y<(cross?4:2);++y)truth.set(2,y,NORTH,Wall::Open);
  for(int x=0;x<4;++x)truth.set(x,2,EAST,Wall::Open);
  auto ph=sim::perturbed(p,0,1);ph.sideSensors=true;
  sim::World world(truth,ph);ObservedPlatform io(world,p,ph,1);io.pauseOnSave=true;io.place(2,0,NORTH);
  Mouse mouse(io,p);auto r=mouse.autonomous();
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
  CHECK(event(io,"phase=DIAGONAL "));
  printf("Diagonal/curvas abiertas: completo=%d diagonal=%d contactos=%d\n",r.autonomousComplete,event(io,"phase=DIAGONAL "),io.contacts());
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

int main(int argc,char** argv) {
  if(argc>1 && !strcmp(argv[1],"--bench"))return benchmark(argc>2?atoi(argv[2]):8);
  policies();mission(false);mission(true);mission(true,true);fatalFault();
  communicationRepair(true);communicationRepair(false);diagonalMission();
  printf("Autonomia: %d comprobaciones, %d fallos\n",checks,failures);
  return failures?1:0;
}
