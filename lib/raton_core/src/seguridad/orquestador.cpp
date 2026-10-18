#include "seguridad/orquestador.h"

#include <math.h>

namespace raton {

void OrquestadorMovimiento::empezarTramo(uint32_t ms) {
  rubSince_ = 0;
  movingAt_ = ms;
  rubs_ = 0;
  pushPos_ = -1;
}

void OrquestadorMovimiento::trasRecuperar(uint32_t ms, float pos) {
  rubSince_ = 0;
  movingAt_ = ms;
  pushPos_ = -1;
  (void)pos;
}

DecisionMovimiento OrquestadorMovimiento::decidir(const EntradaMovimiento& in) {
  DecisionMovimiento d{in.remaining, false, 1e9f, Seguridad::Ok};
  // Regla de seguridad 1: atasco (con potencia pero sin moverse).
  if (in.speed > 20.0f) movingAt_ = in.ms;
  if (in.ms - movingAt_ > p_.stallMs) {
    d.seguridad = Seguridad::Pegado;
    return d;
  }
  // Regla de seguridad 2: empuje contra una pared (encoders avanzan, el frontal no se acerca).
  if (in.front >= 0 && in.front < in.wallNearMm) {
    if (pushPos_ < 0) {
      pushPos_ = in.pos;
      pushFront_ = in.front;
    } else if (in.pos - pushPos_ > p_.pushCheckMm) {
      if (pushFront_ - in.front < p_.pushMinCloseMm) {
        d.seguridad = Seguridad::Pegado;
        return d;
      }
      pushPos_ = in.pos;
      pushFront_ = in.front;
    }
  } else {
    pushPos_ = -1;
  }
  // Regla de seguridad 3: roce lateral o torcido sostenido.
  const bool rubbing = (in.sideL >= 0 && in.sideL < p_.sideCrashMm) || (in.sideR >= 0 && in.sideR < p_.sideCrashMm) ||
                       fabsf(in.skewDeg) > p_.skewLimitDeg;
  if (!rubbing) {
    rubSince_ = 0;
  } else if (rubSince_ == 0) {
    rubSince_ = in.ms ? in.ms : 1;
  } else if (in.ms - rubSince_ > p_.rubMs) {
    d.seguridad = (++rubs_ >= p_.maxRubs) ? Seguridad::Pegado : Seguridad::Roce;
    rubSince_ = 0;
    return d;
  }
  // Regla de frenado: la pared de delante manda sobre los encoders cerca del final del tramo.
  if (in.front >= 0 && in.front < in.wallNearMm) {
    if (in.remaining > in.front + in.cellMm / 2) {
      d.paredInesperada = true;
      d.remaining = static_cast<float>(in.front - in.stopMm);
    } else if (in.remaining < in.cellMm) {
      d.remaining = static_cast<float>(in.front - in.stopMm);
    }
  }
  d.vmaxFrenado = fmaxf(sqrtf(2.0f * p_.accel * fmaxf(d.remaining, 0.0f)), p_.minSpeed);
  return d;
}

}  // namespace raton
