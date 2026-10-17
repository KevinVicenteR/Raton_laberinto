#include "motion_safety.h"
#include <algorithm>

namespace raton {
namespace {
float cross(Point2 a, Point2 b, Point2 c) {
  return (b.x-a.x)*(c.y-a.y)-(b.y-a.y)*(c.x-a.x);
}
bool on(Point2 a, Point2 b, Point2 p) {
  return p.x >= std::min(a.x,b.x)-0.001f && p.x <= std::max(a.x,b.x)+0.001f &&
         p.y >= std::min(a.y,b.y)-0.001f && p.y <= std::max(a.y,b.y)+0.001f;
}
bool intersects(Point2 a, Point2 b, Point2 c, Point2 d) {
  const float x=cross(a,b,c), y=cross(a,b,d), z=cross(c,d,a), w=cross(c,d,b);
  if (((x>0 && y<0)||(x<0 && y>0)) && ((z>0 && w<0)||(z<0 && w>0))) return true;
  return (std::fabs(x)<0.001f && on(a,b,c)) || (std::fabs(y)<0.001f && on(a,b,d)) ||
         (std::fabs(z)<0.001f && on(c,d,a)) || (std::fabs(w)<0.001f && on(c,d,b));
}
bool clearPose(const ObservedSegment* walls, int count, const BodyGeometry& b,
               float x, float y, float angle, float margin) {
  const float c=std::cos(angle), s=std::sin(angle), half=b.width/2+margin;
  Point2 p[4];
  const float fx[4]={b.front+margin,b.front+margin,-b.back-margin,-b.back-margin};
  const float fy[4]={half,-half,-half,half};
  for (int i=0;i<4;++i) p[i]={x+fx[i]*c-fy[i]*s,y+fx[i]*s+fy[i]*c};
  for (int k=0;k<count;++k) {
    if(!std::isfinite(walls[k].a.x)||!std::isfinite(walls[k].a.y)||
       !std::isfinite(walls[k].b.x)||!std::isfinite(walls[k].b.y))return false;
    for (int i=0;i<4;++i) if (intersects(p[i],p[(i+1)%4],walls[k].a,walls[k].b)) return false;
    // A short wall entirely inside the body has no edge intersection.
    const Point2 q=walls[k].a;
    const float qx=(q.x-x)*c+(q.y-y)*s, qy=-(q.x-x)*s+(q.y-y)*c;
    if(qx>=-b.back-margin && qx<=b.front+margin && std::fabs(qy)<=half) return false;
  }
  return true;
}
}
int observedWalls(const SensorObservation s[3], const BodyGeometry& b, ObservedSegment walls[3]) {
  int n=0;
  for(int i=0;i<3;++i) {
    if(!s[i].valid || !std::isfinite(s[i].confidence) || !std::isfinite(s[i].ageMs) || !std::isfinite(s[i].mm) || s[i].mm<0 || s[i].mm>140 ||
       s[i].ageMs>100 || s[i].confidence<0.5f) continue;
    const float d=s[i].mm;
    // Only a short patch around the beam impact is supported by this observation.
    const float patch=15;
    if(i==1) walls[n++]={{b.frontAhead+d,-patch},{b.frontAhead+d,patch}};
    else {
      const float y=(i==0?1:-1)*(b.sensorWidth/2+d);
      walls[n++]={{b.sideAhead-patch,y},{b.sideAhead+patch,y}};
    }
  }
  return n;
}
bool sweptTurnClear(const ObservedSegment* walls,int count,const BodyGeometry& b,
                    float degrees,float split,float advance,float entry,float margin) {
  if(!std::isfinite(degrees)||!std::isfinite(entry)||std::fabs(degrees)!=90 || count<0 || count>3 ||
     !std::isfinite(split)||split<=0||split>=90||!std::isfinite(advance)||advance<0||advance>300||
     !std::isfinite(margin)||margin<0||margin>30||std::fabs(entry)>100||
     !std::isfinite(b.front)||!std::isfinite(b.back)||!std::isfinite(b.width)||b.front<=0||b.back<0||b.width<=0) return false;
  const float sign=degrees>0?1:-1, rad=0.01745329252f;
  // Sampling every <=1 mm or <=1 degree, with a 3 mm inflated envelope by default.
  const int steps=static_cast<int>(std::ceil(std::fabs(entry)));
  for(int i=0;i<=steps;++i) if(!clearPose(walls,count,b,steps?entry*i/steps:0,0,0,margin)) return false;
  for(int i=0;i<=static_cast<int>(std::ceil(split));++i)
    if(!clearPose(walls,count,b,entry,0,sign*split*i/std::ceil(split)*rad,margin)) return false;
  const float a=sign*split*rad;
  const int linear=static_cast<int>(std::ceil(advance));
  for(int i=0;i<=linear;++i) {
    const float d=linear?advance*i/linear:0;
    if(!clearPose(walls,count,b,entry+d*std::cos(a),d*std::sin(a),a,margin)) return false;
  }
  const float x=entry+advance*std::cos(a),y=advance*std::sin(a);
  for(int i=0;i<=static_cast<int>(std::ceil(90-split));++i)
    if(!clearPose(walls,count,b,x,y,sign*(split+(90-split)*i/std::ceil(90-split))*rad,margin)) return false;
  return true;
}
TurnAssessment assessTurn(const SensorObservation s[3],const BodyGeometry& b,float degrees,
                          float split,float advance,float margin) {
  if(!std::isfinite(b.sideAhead)||!std::isfinite(b.frontAhead)||!std::isfinite(b.sensorWidth)||
     b.sensorWidth<=0)return {false,false,0};
  ObservedSegment walls[3]; const int n=observedWalls(s,b,walls);
  // Nominal first. Corrections are controller movements, never altered physical dimensions.
  const float entries[]={0,-5,-10,-15,5,10,15};
  for(float e:entries) if(sweptTurnClear(walls,n,b,degrees,split,advance,e,margin)) return {true,n>0,e};
  return {false,n>0,0};
}
}  // namespace raton
