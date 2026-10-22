// Filtro de Kalman lateral de 2 estados: desplazamiento respecto al centro del pasillo (y, mm, + =
// izquierda) y error del rumbo del giroscopio respecto a la cuadricula (b, rad).
//
// Por que: con sensores a 45 grados la suma izquierda + derecha no dice nada del rumbo, pero al avanzar
// el desplazamiento cambia como v * sin(rumbo real). Si y crece aunque el giroscopio diga "recto", el
// giroscopio esta torcido. Asi se corrige la deriva del sesgo y el error de escala de cada giro (1 % de
// escala = ~1 grado por giro), que de otro modo deja al raton rozando una pared.
#pragma once

namespace raton {

class LateralFilter {
 public:
  void setRobust(bool on) { robust_ = on; candidateCount_ = 0; }
  void reset(float y, float varY, float varB) {
    rejects_ = candidateCount_ = 0;
    y_ = y;
    b_ = 0;
    p00_ = varY;
    p01_ = 0;
    p11_ = varB;
  }
  // Avance ds (mm) con rumbo estimado psi (rad, giroscopio - cuadricula).
  void predict(float ds, float psi) {
    y_ += ds * (psi + b_);
    const float a = ds < 0 ? -ds : ds;
    p00_ += 2.0f * ds * p01_ + ds * ds * p11_ + qY * a;
    p01_ += ds * p11_;
    p11_ += qB * a;
  }
  // Medida del desplazamiento con varianza r. Devuelve false si se descarta por incoherente.
  bool update(float z, float r) {
    const float s = p00_ + r;
    const float inn = z - y_;
    if (inn * inn > 9.0f * s + 100.0f) {
      // Varias medidas seguidas que no cuadran: el filtro estaba equivocado, empezar desde la medida.
      const bool consistent = relocationConsistent(z);
      if (rejects_ < 4) ++rejects_;
      if (rejects_ < 4 || !consistent) return false;
      setY(z, r);
      rejects_ = 0;
      return true;
    }
    rejects_ = 0;
    candidateCount_ = 0;
    const float k0 = p00_ / s, k1 = p01_ / s;
    y_ += k0 * inn;
    b_ += k1 * inn;
    const float p00 = p00_, p01 = p01_;
    p00_ = (1 - k0) * p00;
    p01_ = (1 - k0) * p01;
    p11_ -= k1 * p01;
    return true;
  }
  // Medida general (sonda): lectura con innovacion inn que cambia hy por mm de y y hb por rad de rumbo.
  // Devuelve false si se descarta; dy, db = correccion aplicada.
  bool updateLinear(float inn, float hy, float hb, float r, float& dy, float& db) {
    dy = db = 0;
    const float py = p00_ * hy + p01_ * hb, pb = p01_ * hy + p11_ * hb;  // P * H'
    const float s = hy * py + hb * pb + r;
    if (inn * inn > 9.0f * s + 100.0f) {
      const bool usable = hy >= 0.5f || hy <= -0.5f;
      const bool consistent = usable && relocationConsistent(y_ + inn / hy);
      if (rejects_ < 4) ++rejects_;
      if (rejects_ < 4 || !consistent || !usable) return false;
      // Varias seguidas que no cuadran: el filtro estaba equivocado, empezar desde la medida.
      dy = inn / hy;
      setY(y_ + dy, r / (hy * hy));
      rejects_ = 0;
      return true;
    }
    rejects_ = 0;
    candidateCount_ = 0;
    const float k0 = py / s, k1 = pb / s;
    dy = k0 * inn;
    db = k1 * inn;
    y_ += dy;
    b_ += db;
    p00_ -= k0 * py;
    p01_ -= k0 * pb;
    p11_ -= k1 * pb;
    return true;
  }
  // Tras girar: el rumbo puede haber ganado error (escala del giroscopio).
  void addHeadingVariance(float v) { p11_ += v; }
  // Saca el error de rumbo estimado (para corregir la cuadricula) y lo deja a cero.
  float takeBias() {
    const float b = b_;
    b_ = 0;
    return b;
  }
  float y() const { return y_; }
  float bias() const { return b_; }
  float varY() const { return p00_; }
  void setY(float y, float varY) {
    y_ = y;
    p00_ = varY;
    p01_ = 0;
  }

  float qY = 0.02f;   // mm^2 por mm recorrido (patinar, holguras)
  float qB = 2e-6f;   // rad^2 por mm recorrido (deriva del giroscopio)

 private:
  // Four rejected samples are not evidence of displacement unless their
  // proposed positions agree. Alternating reflections must not relocate us.
  bool relocationConsistent(float z) {
    if (!robust_) return true;
    const float delta = z - candidate_;
    if (!candidateCount_ || delta > 6 || delta < -6) { candidate_ = z; candidateCount_ = 1; }
    else { if (candidateCount_ < 4) ++candidateCount_; candidate_ += delta / candidateCount_; }
    return candidateCount_ >= 4;
  }
  bool robust_ = false;
  float candidate_ = 0;
  int candidateCount_ = 0;
  float y_ = 0, b_ = 0;
  int rejects_ = 0;
  float p00_ = 1e4f, p01_ = 0, p11_ = 1e-3f;
};

}  // namespace raton

