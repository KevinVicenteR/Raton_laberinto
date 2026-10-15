#include "motion_safety.h"
#include "risk_model.h"
extern "C" {
float raton_risk(const float* x) { return raton::predictRisk(raton::kRiskModel,x); }
void raton_features(const float* values,float speed,float yaw,float heading,float pending,int action,float* out) {
  raton::RiskFrame h[3];
  for(int t=0;t<3;++t) for(int i=0;i<3;++i) {
    const int k=(t*3+i)*4;
    h[t].sensor[i]={values[k],values[k+2],values[k+3],values[k+1]>0};
  }
  raton::riskFeatures(h,speed,yaw,heading,pending,action,out);
}
// values are L,F,R tuples (mm,valid,confidence,age); body order is documented in navigation.py.
float raton_turn(const float* values,const float* body,float deg,float split,float advance) {
  raton::SensorObservation s[3];
  for(int i=0;i<3;++i)s[i]={values[4*i],values[4*i+2],values[4*i+3],values[4*i+1]>0};
  raton::BodyGeometry b{body[0],body[1],body[2],body[3],body[4],body[5]};
  const auto a=raton::assessTurn(s,b,deg,split,advance);
  return a.feasible?a.entryMm:10000.f;
}
}
