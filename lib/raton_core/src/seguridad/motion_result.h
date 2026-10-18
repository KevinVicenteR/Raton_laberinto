// Resultado de cada maniobra (se registra en la telemetria como linea MOTION).
#pragma once

namespace raton {

enum class MotionReason { Complete, Interrupted, TurnError, ObservedObstacle, InvalidObservation, Aborted };

struct MotionResult {
  bool completed = false;
  float actualMm = 0;
  float angularErrorDeg = 0;
  MotionReason reason = MotionReason::Interrupted;
};

}  // namespace raton
