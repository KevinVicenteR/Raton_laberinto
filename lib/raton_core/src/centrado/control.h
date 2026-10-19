#pragma once
#include <math.h>

namespace raton {
struct SteeringParams {
  float LATERAL_DEADBAND_MM;
  float KD_YAW;
  float KP_YAW;
  float WALL_RATE_DAMP;
  float MAX_WALL_CORRECTION_DEG;
  float YAW_TARGET_SMOOTH;
  float MAX_STEER;
};

inline float steering(float v, float dt, bool walls, float offset, float rate, float conf,
                      int mode, float kpSteer, float kdSteer, float kpWall, float heading,
                      float gyroRate, float& headingRef, float& yawTarget, const SteeringParams& p) {
  const auto clampf = [](float value, float lo, float hi) { return value < lo ? lo : (value > hi ? hi : value); };
  // Direccion en capas:
  //  - con paredes: desviacion lateral con ganancias escaladas por la confianza de los laseres;
  //  - sin paredes (o laseres poco fiables): mantener el rumbo con el giroscopio (KP_YAW).
  // En ambos casos el giroscopio amortigua (KD_YAW). Positivo = girar a la izquierda.
  float steer;
  if (walls) {
    if (fabsf(offset) <= p.LATERAL_DEADBAND_MM) {
      offset = 0;
    } else {
      offset -= offset > 0 ? p.LATERAL_DEADBAND_MM : -p.LATERAL_DEADBAND_MM;
    }
  }
  if (mode == 1) {
    // PID directo sobre la desviacion (KP_STEER, KD_STEER), escalado por la confianza.
    if (walls) {
      steer = conf * (kpSteer * offset + kdSteer * rate * dt) - p.KD_YAW * gyroRate;
      headingRef = heading;
    } else {
      steer = p.KP_YAW * (headingRef - heading) - p.KD_YAW * gyroRate;
    }
  } else {
    // Cascada: la desviacion fija el rumbo objetivo (con menos fuerza si los laseres son poco fiables)
    // y el PID del giroscopio lo sigue.
    float target = 0;
    if (walls) {
      // La deriva lateral del Kalman anticipa el cruce del centro. Limitarla evita
      // que un cambio de pared o un valor atipico domine el rumbo.
      const float speedGain = clampf(132.0f / fmaxf(v, 80.0f), 0.5f, 1.0f);
      target = clampf(conf * speedGain * (kpWall * offset +
                      p.WALL_RATE_DAMP * clampf(rate, -40.0f, 40.0f)),
                      -p.MAX_WALL_CORRECTION_DEG, p.MAX_WALL_CORRECTION_DEG);
    }
    yawTarget += p.YAW_TARGET_SMOOTH * (target - yawTarget);
    steer = p.KP_YAW * (yawTarget - heading) - p.KD_YAW * gyroRate;
  }
  steer = clampf(steer, -p.MAX_STEER, p.MAX_STEER);
  return steer;
}
}  // namespace raton
