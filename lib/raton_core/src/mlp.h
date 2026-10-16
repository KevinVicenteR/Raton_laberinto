// Perceptron de dos capas ocultas (ReLU) y salida lineal, para las redes de control y de logica.
// Los pesos los genera tools/nets.py (control_model.h, logic_model.h).
#pragma once
#include <math.h>

namespace raton {

template <int I, int H1, int H2, int O>
struct Mlp {
  static constexpr int kIn = I;
  static constexpr int kOut = O;
  float w1[H1][I], b1[H1], w2[H2][H1], b2[H2], w3[O][H2], b3[O];
};

// Devuelve false si alguna entrada o salida no es un numero (quien llama usa entonces el control normal).
template <int I, int H1, int H2, int O>
bool mlpForward(const Mlp<I, H1, H2, O>& m, const float* x, float* out) {
  for (int i = 0; i < I; ++i) if (!isfinite(x[i])) return false;
  float a[H1], b[H2];
  for (int i = 0; i < H1; ++i) {
    float v = m.b1[i];
    for (int j = 0; j < I; ++j) v += m.w1[i][j] * x[j];
    a[i] = v > 0 ? v : 0;
  }
  for (int i = 0; i < H2; ++i) {
    float v = m.b2[i];
    for (int j = 0; j < H1; ++j) v += m.w2[i][j] * a[j];
    b[i] = v > 0 ? v : 0;
  }
  for (int i = 0; i < O; ++i) {
    float v = m.b3[i];
    for (int j = 0; j < H2; ++j) v += m.w3[i][j] * b[j];
    if (!isfinite(v)) return false;
    out[i] = v;
  }
  return true;
}

}  // namespace raton
