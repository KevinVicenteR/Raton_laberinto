#pragma once
#include "motion_safety.h"

namespace raton {
constexpr int kRiskInputs=43;
struct RiskFrame { SensorObservation sensor[3]; };
struct RiskModel {
  float w1[32][43], b1[32], w2[16][32], b2[16], w3[16], b3;
};
void riskFeatures(const RiskFrame history[3],float speed,float yawRate,float headingError,
                  float pending,int action,float out[43]);
float predictRisk(const RiskModel& model,const float features[43]);
}  // namespace raton
