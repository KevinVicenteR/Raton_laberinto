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

void Motion::loadLearned() {
  motorBalance_ = balanceRef_ = p_.motorBalance;
  const float saved[4] = {p_.startBoostLF, p_.startBoostLR, p_.startBoostRF, p_.startBoostRR};
  for (int i = 0; i < 4; ++i) boost_[i] = boostRef_[i] = saved[i];
  ffScale_ = 1.0f;
  gyroScale_ = p_.gyroScale;
}

void Motion::reset(const Sensors& s) {
  if (balanceRef_ != p_.motorBalance) motorBalance_ = balanceRef_ = p_.motorBalance;
  const float savedBoost[4] = {p_.startBoostLF, p_.startBoostLR, p_.startBoostRF, p_.startBoostRR};
  for (int i = 0; i < 4; ++i)
    if (boostRef_[i] != savedBoost[i]) boost_[i] = boostRef_[i] = savedBoost[i];
  for (int i = 0; i < 2; ++i) { breakaway_[i].reset(); wheelSpeed_[i] = 0; }
  wheelLast_[0] = s.encLeftMm;
  wheelLast_[1] = s.encRightMm;
  sensorFault_ = encoderFault_ = gyroDisagreement_ = false;
  updateT_ = measureT_ = s.t;
  controlDt_ = p_.dt;
  for (int i = 0; i < 2; ++i) oppositeT_[i] = previousDuty_[i] = 0;
  lastGyroT_ = gyroProgressT_ = s.t;
  gyroProgress_ = s.gyroDeg;
  encProgress_ = 0;
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
  rotDerivative_ = 0;
  iF_ = iR_ = 0;
  satF_ = 0;
  satR_ = 0;
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
  float dt = s.t - measureT_;
  if (dt <= 0) return; // hold() puede recibir la misma muestra: no inventar una velocidad cero
  if (fabsf(dt - p_.dt) < 0.05f * p_.dt) dt = p_.dt;
  dt = clampf(dt, 0.0002f, 0.2f);
  measureT_ = s.t;
  const float alpha = clampf(0.3f * dt / p_.dt, 0.02f, 0.9f);
  const float wheel[2] = {s.encLeftMm, s.encRightMm};
  for (int i = 0; i < 2; ++i) {
    wheelSpeed_[i] += alpha * ((wheel[i] - wheelLast_[i]) / dt - wheelSpeed_[i]);
    wheelLast_[i] = wheel[i];
  }
  vMeas_ += alpha * ((pos_ - lastPos_) / dt - vMeas_);
  wMeas_ += alpha * ((angle_ - lastAngle_) / dt - wMeas_);
  lastPos_ = pos_;
  lastAngle_ = angle_;
}

void Motion::moveTo(float end, float vmax, float vEnd) {
  end_ = end;
  vmax_ = p_.precisionMode && vmax > 300 ? 300 : vmax;
  vEnd_ = vEnd > vmax_ ? vmax_ : vEnd;
  done_ = false;
  dir_ = end >= ideal_ ? 1.0f : -1.0f;
  // Si se cambia de sentido con el raton en marcha, primero frena (la velocidad en "u" seria negativa).
}

void Motion::turnTo(float target, float rate, float accel) {
  angleTarget_ = target;
  rate_ = p_.precisionMode && rate > 360 ? 360 : rate;
  rotAccel_ = p_.precisionMode && accel > 1800 ? 1800 : accel;
  rotDone_ = false;
  steer_ = 0;
  rotDir_ = target >= idealAngle_ ? 1.0f : -1.0f;
}

void Motion::hold(const Sensors& s) {
  measure(s);
  gyroDisagreement_ = false;
  gyroProgress_ = s.gyroDeg;
  gyroProgressT_ = s.t;
  encProgress_ = ((s.encRightMm - encR0_) - (s.encLeftMm - encL0_)) / p_.trackMm * kDeg;
  ideal_ = end_ = pos_;
  v_ = a_ = 0;
  done_ = true;
  vEnd_ = 0;
  idealAngle_ = angle_;
  w_ = alpha_ = 0;
  rotDone_ = true;
  lastErrF_ = lastErrR_ = 0;
  rotDerivative_ = 0;
  iF_ = iR_ = 0;
  satF_ = satR_ = 0;
}

void Motion::update(const Sensors& s, float& dutyL, float& dutyR) {
  // Una IMU sin muestras no debe quedar sustituida indefinidamente por encoders.
  // Si llegan muestras validas pero no hay giro mientras las ruedas giran, puede ser
  // patinaje/atasco: es una discrepancia recuperable, no prueba de una IMU averiada.
  const float encAngle = ((s.encRightMm - encR0_) - (s.encLeftMm - encL0_)) / p_.trackMm * kDeg;
  if (s.gyroFresh && isfinite(s.gyroDeg)) {
    lastGyroT_ = s.t;
    if (fabsf(s.gyroDeg - gyroProgress_) > 0.1f) {
      gyroProgress_ = s.gyroDeg;
      gyroProgressT_ = s.t;
      encProgress_ = encAngle;
      gyroDisagreement_ = false;
    } else if (s.t - gyroProgressT_ > 0.2f && fabsf(encAngle - encProgress_) > 20.0f) {
      gyroDisagreement_ = true;
    }
  }
  if (!isfinite(s.gyroDeg) || s.t - lastGyroT_ > 0.1f) sensorFault_ = true;
  if (!isfinite(s.encLeftMm) || !isfinite(s.encRightMm)) sensorFault_ = encoderFault_ = true;
  if (sensorFault_) {
    dutyL = dutyR = 0;
    return;
  }
  float elapsed = s.t - updateT_;
  updateT_ = s.t;
  if (elapsed <= 0 || fabsf(elapsed - p_.dt) < 0.05f * p_.dt) elapsed = p_.dt;
  controlDt_ = clampf(elapsed, 0.0002f, 0.02f);
  measure(s);
  const float dt = controlDt_;
  const float accel = p_.precisionMode && p_.accel > 1500 ? 1500 : p_.accel;
  const float decel = p_.precisionMode && p_.decel > 2000 ? 2000 : p_.decel;

  // ---- Perfil de avance ----
  const float vPrev = v_;
  if (!done_) {
    float u = dir_ * v_;  // velocidad en el sentido del movimiento
    if (u < 0) {
      // Va en sentido contrario: frenar primero.
      u += 1.5f * decel * dt;
      if (u > 0) u = 0;
    } else {
      // No pedir mas de lo que los motores dan (bateria baja, motores lentos): con duty al maximo hace un
      // ciclo, la consigna no se adelanta a lo medido. Si no, la consigna se escapa, el error crece y parece
      // un atasco a toda velocidad. Tope de crucero con el modelo aprendido y margen para corregir el rumbo.
      const float vTop = topSpeed();
      u = profileStep(u, dir_ * (end_ - ideal_), vmax_ < vTop ? vmax_ : vTop, vEnd_, accel, decel, dt);
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
  float dR = (errR - lastErrR_) / dt;
  if (stableControl_) {
    // IMU arrivals and encoder quantization must not become 500 Hz torque kicks.
    // Faster damping in pivots retains their braking response.
    const float tau = rotDone_ ? 0.025f : 0.012f;
    rotDerivative_ += dt / (tau + dt) * (dR - rotDerivative_);
    dR = rotDerivative_;
  }
  lastErrF_ = errF;
  lastErrR_ = errR;

  // ---- Modelo del motor por rueda ----
  const float half = 0.5f * p_.trackMm / kDeg;  // mm de rueda por grado
  const float vl = v_ - w_ * half, vr = v_ + w_ * half;
  const float al = a_ - alpha_ * half, ar = a_ + alpha_ * half;
  const float requested[2] = {vl, vr};
  for (int i = 0; i < 2; ++i) {
    const float cmd = requested[i];
    const bool opposite = fabsf(cmd) > 40 && fabsf(previousDuty_[i]) > 0.12f && previousDuty_[i] * cmd > 0 &&
                          wheelSpeed_[i] * cmd < -15 * fabsf(cmd);
    oppositeT_[i] = opposite ? oppositeT_[i] + dt : 0;
    if (oppositeT_[i] > 0.15f) sensorFault_ = encoderFault_ = true;
  }
  if (sensorFault_) { dutyL = dutyR = 0; return; }

  // Aceleraciones enormes de un paso (cambios de consigna) no se pasan al modelo.
  const float alc = clampf(al, -3.0f * p_.accel, 3.0f * p_.accel);
  const float arc = clampf(ar, -3.0f * p_.accel, 3.0f * p_.accel);
  // Rozamiento (kS) en rampa con la velocidad pedida, no a saltos: al pasar de avanzar a retroceder (o al
  // arrancar) no hay tiron.
  auto ff = [&](float v, float a) { return ffScale_ * (p_.kV * v + p_.kA() * a) + p_.kS * clampf(v / 20.0f, -1.0f, 1.0f); };
  // Integral pequeno con limite: quita el error que deja el rozamiento al final de giros y paradas.
  // Conditional integration: stop winding up when saturation opposes this error;
  // allow integration immediately when the error would release the saturation.
  if (!stableControl_ || satF_ * errF <= 0) iF_ = clampf(iF_ + errF * dt, -0.5f, 0.5f);
  if (!stableControl_ || satR_ * errR <= 0) iR_ = clampf(iR_ + errR * dt, -0.5f, 0.5f);
  const float fbF = p_.kpFwd * errF + p_.kdFwd * dF + p_.kiFwd * iF_;
  const float fbR = p_.kpRot * errR + p_.kdRot * dR + p_.kiRot * iR_;
  // Motores aprendidos en marcha: a velocidad de crucero, lo que tiene que poner la realimentacion es lo que
  // le falta (o sobra) al modelo. Se pasa poco a poco a la escala del modelo (bateria, motores distintos de
  // los parametros), sin calibrar a mano. Constante de tiempo ~2 s de crucero.
  if (p_.learnMotor && fabsf(v_) > 120.0f && fabsf(a_) < 200.0f && rotDone_ && fabsf(errF) < 20.0f && satF_ == 0) {
    const float rel = fbF / (p_.kV * fabsf(v_)) * (v_ > 0 ? 1.0f : -1.0f);
    ffScale_ = clampf(ffScale_ + 0.5f * rel * ffScale_ * dt, 0.5f, 2.0f);
  }
  // Compensacion diferencial: aprender solo en rectas estables, lejos de saturacion o atasco.
  if (p_.learnMotor && fabsf(v_) > 120 && fabsf(a_) < 200 && rotDone_ &&
      fabsf(wMeas_) < 5 && fabsf(errR) < 2 && fabsf(errF) < 10 && satF_ == 0) {
    motorBalance_ = clampf(motorBalance_ + 0.02f * fbR / (p_.kV * v_) * dt, -0.25f, 0.25f);
  }
  float l = (1.0f - motorBalance_) * ff(vl, alc) + fbF - fbR;
  float r = (1.0f + motorBalance_) * ff(vr, arc) + fbF + fbR;
  // Zona muerta: con la rueda casi parada, una correccion pequena no la mueve (rozamiento estatico). Se
  // suma kS en el sentido de la correccion para que el error final quede en ~1 mm / ~0.5 grados. Sin saltos:
  // pasa poco a poco del kS que pone el modelo (segun la velocidad pedida) al de la correccion, y este
  // crece en rampa hasta 0.02 de duty (no tiembla parado).
  auto deadband = [&](float out, float vcmd) {
    const float moving = fabsf(clampf(vcmd / 20.0f, -1.0f, 1.0f));
    return out + p_.kS * (1.0f - moving) * clampf(out / 0.02f, -1.0f, 1.0f);
  };
  l = deadband(l, vl);
  r = deadband(r, vr);
  // Ayuda suave por rueda y sentido, tambien para terminar una correccion significativa.
  // Con consignas cumplidas no hay pulso.
  auto boostCommand = [](float vcmd, float wheelError) {
    if (fabsf(vcmd) >= 25) return vcmd;
    if (fabsf(wheelError) < 0.7f) return 0.0f;
    return wheelError > 0 ? 30.0f : -30.0f;
  };
  const float boostL = boostCommand(vl, errF - half * errR);
  const float boostR = boostCommand(vr, errF + half * errR);
  const float helpL = breakaway_[0].update(boostL, wheelSpeed_[0], dt, p_.learnMotor, boost_[boostL < 0 ? 1 : 0]);
  const float helpR = breakaway_[1].update(boostR, wheelSpeed_[1], dt, p_.learnMotor, boost_[boostR < 0 ? 3 : 2]);
  if (l * boostL > 0) l += helpL;
  if (r * boostR > 0) r += helpR;

  // Saturacion dando prioridad al giro (mantener el rumbo vale mas que la velocidad).
  float common = 0.5f * (l + r);
  float diff = 0.5f * (r - l);
  satR_ = diff > p_.maxDuty ? 1.0f : diff < -p_.maxDuty ? -1.0f : 0.0f;
  diff = clampf(diff, -p_.maxDuty, p_.maxDuty);
  const float room = p_.maxDuty - fabsf(diff);
  satF_ = common > room ? 1.0f : (common < -room ? -1.0f : 0.0f);
  common = clampf(common, -room, room);
  dutyL = common - diff;
  dutyR = common + diff;
  previousDuty_[0] = dutyL;
  previousDuty_[1] = dutyR;
}

}  // namespace raton

