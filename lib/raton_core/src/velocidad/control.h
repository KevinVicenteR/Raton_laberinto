#pragma once

namespace raton {
struct VelocityParams {
  float PWM_MIN_RUN;
  float KFF_SPEED;
  float KP_SPEED;
  float KI_SPEED;
  float PWM_MIN_MOVE;
  float PWM_MAX;
  float MAX_STEER;
};

inline float speedOutput(float v, float speed, float& integral, float dt, const VelocityParams& p) {
  const auto clampf = [](float value, float lo, float hi) { return value < lo ? lo : (value > hi ? hi : value); };
  const float err = v - speed;
  integral = clampf(integral + err * dt, -50.0f, 50.0f);
  // Los N20 no se mueven por debajo de cierta potencia: en marcha siempre llevan PWM_MIN_RUN y sobre
  // eso se suma lo que pide la velocidad. Sin esto avanzaba a tirones.
  float base = 0;
  if (v > 0) {
    base = p.PWM_MIN_RUN + p.KFF_SPEED * v + p.KP_SPEED * err + p.KI_SPEED * integral;
    if (speed < 20.0f && base < p.PWM_MIN_MOVE) base = p.PWM_MIN_MOVE;  // parado: empujon para arrancar
    // En frenada el PI debe poder bajar del PWM de mantenimiento: forzarlo
    // mantenia las ruedas empujando aunque la velocidad ya excediera la consigna.
    base = clampf(base, 0.0f, p.PWM_MAX - p.MAX_STEER);
  }

  return base;
}
}  // namespace raton
