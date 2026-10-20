#include "raton/motion.h"

#include <math.h>

namespace raton {

namespace {

const float kDeg = 57.2957795f;

float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

// Un paso del perfil trapezoidal en "u" (posicion en la direccion del movimiento, siempre creciente).
// rem = lo que falta, uEnd = velocidad final, devuelve la nueva velocidad.
float profileStep(float v, float rem, float vmax, float vEnd, float accel, float decel, float dt) {
  float allowed = vEnd * vEnd + 2.0f * decel * (rem > 0 ? rem : 0);
  allowed = sqrtf(allowed);
  const float target = allowed < vmax ? allowed : vmax;
  if (v < target) {
    v += accel * dt;
    if (v > target) v = target;
  } else {
    // Frenar algo mas fuerte que el perfil si hace falta, para no pasarse.
    v -= 1.5f * decel * dt;
    if (v < target) v = target;
  }
  return v < 0 ? 0 : v;
}

}  // namespace

void Motion::reset(const Sensors& s) {
  encL0_ = s.encLeftMm;
  encR0_ = s.encRightMm;
  gyroAtSample_ = 0;
  encAngleAtSample_ = 0;
  gyroRaw_ = s.gyroDeg;
  pos_ = lastPos_ = vMeas_ = 0;
  angle_ = lastAngle_ = wMeas_ = 0;
  ideal_ = v_ = a_ = end_ = 0;
  vmax_ = p_.searchSpeed;
  vEnd_ = 0;
  done_ = true;
  lastErrF_ = 0;
  idealAngle_ = w_ = alpha_ = angleTarget_ = steer_ = 0;
  rotDone_ = true;
  lastErrR_ = 0;
  iF_ = iR_ = 0;
  satF_ = 0;
}

void Motion::measure(const Sensors& s) {
  const float l = s.encLeftMm - encL0_;
  const float r = s.encRightMm - encR0_;
  pos_ = 0.5f * (l + r);
  const float encAngle = (r - l) / p_.trackMm * kDeg;
  if (s.gyroFresh) {
    gyroAtSample_ += (s.gyroDeg - gyroRaw_) * gyroScale_;
    gyroRaw_ = s.gyroDeg;
    encAngleAtSample_ = encAngle;
  }
  angle_ = gyroAtSample_ + (encAngle - encAngleAtSample_);
  const float dt = p_.dt;
  vMeas_ += 0.3f * ((pos_ - lastPos_) / dt - vMeas_);
  wMeas_ += 0.3f * ((angle_ - lastAngle_) / dt - wMeas_);
  lastPos_ = pos_;
  lastAngle_ = angle_;
}

void Motion::moveTo(float end, float vmax, float vEnd) {
  end_ = end;
  vmax_ = vmax;
  vEnd_ = vEnd;
  done_ = false;
  dir_ = end >= ideal_ ? 1.0f : -1.0f;
  // Si se cambia de sentido con el raton en marcha, primero frena (la velocidad en "u" seria negativa).
}

void Motion::turnTo(float target, float rate, float accel) {
  angleTarget_ = target;
  rate_ = rate;
  rotAccel_ = accel;
  rotDone_ = false;
  steer_ = 0;
  rotDir_ = target >= idealAngle_ ? 1.0f : -1.0f;
}

void Motion::hold(const Sensors& s) {
  measure(s);
  ideal_ = end_ = pos_;
  v_ = a_ = 0;
  done_ = true;
  vEnd_ = 0;
  idealAngle_ = angle_;
  w_ = alpha_ = 0;
  rotDone_ = true;
  lastErrF_ = lastErrR_ = 0;
  iF_ = iR_ = 0;
}

void Motion::update(const Sensors& s, float& dutyL, float& dutyR) {
  measure(s);
  const float dt = p_.dt;

  // ---- Perfil de avance ----
  const float vPrev = v_;
  if (!done_) {
    float u = dir_ * v_;  // velocidad en el sentido del movimiento
    if (u < 0) {
      // Va en sentido contrario: frenar primero.
      u += 1.5f * p_.decel * dt;
      if (u > 0) u = 0;
    } else {
      // No pedir mas de lo que los motores dan (bateria baja, motores lentos): con duty al maximo hace un
      // ciclo, la consigna no se adelanta a lo medido. Si no, la consigna se escapa, el error crece y parece
      // un atasco a toda velocidad. Tope de crucero con el modelo aprendido y margen para corregir el rumbo.
      const float vTop = topSpeed();
      u = profileStep(u, dir_ * (end_ - ideal_), vmax_ < vTop ? vmax_ : vTop, vEnd_, p_.accel, p_.decel, dt);
      const float uMeas = dir_ * vMeas_;
      if (satF_ * dir_ > 0 && u > uMeas + 40.0f) u = uMeas + 40.0f > 0 ? uMeas + 40.0f : 0;
    }
    v_ = dir_ * u;
    ideal_ += v_ * dt;
    if (dir_ * (end_ - ideal_) <= 0) {
      if (vEnd_ <= 0) {
        ideal_ = end_;
        v_ = 0;
      }
      done_ = true;
    }
  } else if (vEnd_ > 0) {
    ideal_ += v_ * dt;  // sigue a la velocidad final hasta la siguiente orden
  } else {
    v_ = 0;
  }
  a_ = (v_ - vPrev) / dt;

  // ---- Perfil de rumbo ----
  const float wPrev = w_;
  if (!rotDone_) {
    float u = rotDir_ * w_;
    if (u < 0) {
      u += 1.5f * rotAccel_ * dt;
      if (u > 0) u = 0;
    } else {
      u = profileStep(u, rotDir_ * (angleTarget_ - idealAngle_), rate_, 0, rotAccel_, rotAccel_, dt);
    }
    w_ = rotDir_ * u;
    idealAngle_ += w_ * dt;
    if (rotDir_ * (angleTarget_ - idealAngle_) <= 0.01f) {
      idealAngle_ = angleTarget_;
      w_ = 0;
      rotDone_ = true;
    }
  } else {
    // Rumbo fijo + correccion de las paredes, sin saltos (como mucho 60 grados/s).
    const float maxStep = 60.0f * dt;
    const float step = clampf(angleTarget_ + steer_ - idealAngle_, -maxStep, maxStep);
    idealAngle_ += step;
    w_ = 0;  // sin anticipacion: el PD sigue la correccion suave
  }
  // La aceleracion angular solo se anticipa en los giros (el centrado mueve el rumbo a pasitos).
  alpha_ = rotDone_ && wPrev == 0 ? 0 : (w_ - wPrev) / dt;
  if (rotDone_) alpha_ = 0;

  // ---- Errores ----
  const float errF = clampf(ideal_ - pos_, -25.0f, 25.0f);
  const float errR = clampf(idealAngle_ - angle_, -25.0f, 25.0f);
  // Sin acumular un error enorme si algo bloquea la rueda: la consigna no se aleja mas de 25 mm.
  if (ideal_ - pos_ > 25.0f) ideal_ = pos_ + 25.0f;
  if (ideal_ - pos_ < -25.0f) ideal_ = pos_ - 25.0f;
  const float dF = (errF - lastErrF_) / dt;
  const float dR = (errR - lastErrR_) / dt;
  lastErrF_ = errF;
  lastErrR_ = errR;

  // ---- Modelo del motor por rueda ----
  const float half = 0.5f * p_.trackMm / kDeg;  // mm de rueda por grado
  const float vl = v_ - w_ * half, vr = v_ + w_ * half;
  const float al = a_ - alpha_ * half, ar = a_ + alpha_ * half;
  // Aceleraciones enormes de un paso (cambios de consigna) no se pasan al modelo.
  const float alc = clampf(al, -3.0f * p_.accel, 3.0f * p_.accel);
  const float arc = clampf(ar, -3.0f * p_.accel, 3.0f * p_.accel);
  auto ff = [&](float v, float a) {
    float d = ffScale_ * (p_.kV * v + p_.kA() * a);
    if (v > 2.0f) d += p_.kS;
    else if (v < -2.0f) d -= p_.kS;
    return d;
  };
  // Integral pequeno con limite: quita el error que deja el rozamiento al final de giros y paradas.
  iF_ = clampf(iF_ + errF * dt, -0.5f, 0.5f);
  iR_ = clampf(iR_ + errR * dt, -0.5f, 0.5f);
  const float fbF = p_.kpFwd * errF + p_.kdFwd * dF + p_.kiFwd * iF_;
  const float fbR = p_.kpRot * errR + p_.kdRot * dR + p_.kiRot * iR_;
  // Motores aprendidos en marcha: a velocidad de crucero, lo que tiene que poner la realimentacion es lo que
  // le falta (o sobra) al modelo. Se pasa poco a poco a la escala del modelo (bateria, motores distintos de
  // los parametros), sin calibrar a mano. Constante de tiempo ~2 s de crucero.
  if (p_.learnMotor && fabsf(v_) > 120.0f && fabsf(a_) < 200.0f && rotDone_ && fabsf(errF) < 20.0f && satF_ == 0) {
    const float rel = fbF / (p_.kV * fabsf(v_)) * (v_ > 0 ? 1.0f : -1.0f);
    ffScale_ = clampf(ffScale_ + 0.5f * rel * ffScale_ * dt, 0.5f, 2.0f);
  }
  float l = ff(vl, alc) + fbF - fbR;
  float r = ff(vr, arc) + fbF + fbR;
  // Zona muerta: con la rueda casi parada, una correccion pequena no la mueve (rozamiento estatico). Se
  // suma kS en el sentido de la correccion para que el error final quede en ~1 mm / ~0.5 grados.
  auto deadband = [&](float out, float vcmd) {
    if (fabsf(vcmd) > 2.0f) return out;
    if (out > 0.004f) return out + p_.kS;
    if (out < -0.004f) return out - p_.kS;
    return 0.0f;
  };
  l = deadband(l, vl);
  r = deadband(r, vr);

  // Saturacion dando prioridad al giro (mantener el rumbo vale mas que la velocidad).
  float common = 0.5f * (l + r);
  float diff = 0.5f * (r - l);
  diff = clampf(diff, -p_.maxDuty, p_.maxDuty);
  const float room = p_.maxDuty - fabsf(diff);
  satF_ = common > room ? 1.0f : (common < -room ? -1.0f : 0.0f);
  common = clampf(common, -room, room);
  dutyL = common - diff;
  dutyR = common + diff;
}

}  // namespace raton
