// Estado de la ventana de lectura por celda. No depende de Arduino.
#pragma once
#include <stdint.h>

namespace raton {
inline int medianOf(int* v, int n) {
  for (int a = 1; a < n; ++a)
    for (int b = a; b > 0 && v[b] < v[b - 1]; --b) {
      const int t = v[b];
      v[b] = v[b - 1];
      v[b - 1] = t;
    }
  return n >= 2 ? v[n / 2] : (n == 1 ? v[0] : -1);
}


struct CellWindow {
  int winL[24], winR[24], nWinL = 0, nWinR = 0;
  uint32_t winSample = 0;
  bool windowSeen = false;
};
}  // namespace raton
