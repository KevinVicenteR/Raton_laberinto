#pragma once
#include <cmath>

namespace raton {
constexpr bool kMotionGuardEnabled=false;  // Experimental: initial suite showed regressions.

enum class MotionReason { Complete, Interrupted, TurnError, ObservedObstacle, InvalidObservation, Aborted };
struct MotionResult {
  bool completed = false;
  float actualMm = 0;
  float angularErrorDeg = 0;
  MotionReason reason = MotionReason::Interrupted;
};
struct SensorObservation {
  float mm = -1, confidence = 0, ageMs = 1000;
  bool valid = false;
};
struct BodyGeometry {
  // Same measured envelope already used by the physical simulator; not sensor separation.
  float front = 83, back = 17, width = 92;
  float sideAhead = 56, frontAhead = 33, sensorWidth = 90;
};
struct Point2 { float x, y; };
struct ObservedSegment { Point2 a, b; };
struct TurnAssessment { bool feasible; bool observed; float entryMm; };

// Local x points forward, y left. These are finite observed wall patches, not a complete map.
int observedWalls(const SensorObservation sensor[3], const BodyGeometry& body,
                  ObservedSegment walls[3]);
bool sweptTurnClear(const ObservedSegment* walls, int count, const BodyGeometry& body,
                    float degrees, float split, float advance, float entry, float margin);
TurnAssessment assessTurn(const SensorObservation sensor[3], const BodyGeometry& body,
                          float degrees, float split = 70, float advance = 75, float margin = 3);
}  // namespace raton
