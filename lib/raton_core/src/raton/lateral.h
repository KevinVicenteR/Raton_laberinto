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
  void reset(float y, float varY, float varB) {
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
      if (++rejects_ < 4) return false;
      setY(z, r);
      rejects_ = 0;
      return true;
    }
    rejects_ = 0;
    const float k0 = p00_ / s, k1 = p01_ / s;
    y_ += k0 * inn;
    b_ += k1 * inn;
    const float p00 = p00_, p01 = p01_;
    p00_ = (1 - k0) * p00;
    p01_ = (1 - k0) * p01;
    p11_ -= k1 * p01;
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
  float y_ = 0, b_ = 0;
  int rejects_ = 0;
  float p00_ = 1e4f, p01_ = 0, p11_ = 1e-3f;
};

}  // namespace raton
