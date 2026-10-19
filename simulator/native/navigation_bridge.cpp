// Biblioteca nativa para el simulador Python (ctypes): el verificador de barrido de giros en C++.
#include "motion_safety.h"

extern "C" {
// values: tuplas L,F,R (mm, valida, confianza, edad); body en el orden documentado en native.py.
float raton_turn(const float* values, const float* body, float deg, float split, float advance) {
  raton::SensorObservation s[3];
  for (int i = 0; i < 3; ++i) s[i] = {values[4 * i], values[4 * i + 2], values[4 * i + 3], values[4 * i + 1] > 0};
  raton::BodyGeometry b{body[0], body[1], body[2], body[3], body[4], body[5]};
  const auto a = raton::assessTurn(s, b, deg, split, advance);
  return a.feasible ? a.entryMm : 10000.f;
}
}
