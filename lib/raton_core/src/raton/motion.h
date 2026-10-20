// Control de movimiento a 500 Hz.
//  - Avance: perfil trapezoidal hacia una posicion final con velocidad final (0 = parar). Para ir de
//    crucero se pide una posicion muy lejana.
//  - Rumbo: perfil trapezoidal hacia un angulo absoluto (giros) o rumbo fijo + correccion de paredes.
//  - Salida por rueda = modelo del motor (kS, kV, kA) + PD sobre el error de posicion y de rumbo.
// El rumbo medido es el del giroscopio interpolado con los encoders entre muestras (el giroscopio
// manda, los encoders rellenan los 2 ms entre lecturas).
#pragma once

#include "raton/params.h"
#include "raton/platform.h"

namespace raton {

class Motion {
 public:
  explicit Motion(const Params& p) : p_(p) {}

  // Pone todo a cero en la pose actual (llamar con el raton quieto).
  void reset(const Sensors& s);

  // ---- Avance ----
  // Ir hasta la posicion absoluta end (mm de recorrido del eje), sin pasar de vmax, llegando a vEnd.
  void moveTo(float end, float vmax, float vEnd);
  void cruise(float vmax) { moveTo(1e9f, vmax, vmax); }
  void stopAt(float end) { moveTo(end, vmax_, 0); }
  void setMaxSpeed(float vmax) { vmax_ = vmax; }
  float endPosition() const { return end_; }
  bool arrived() const { return done_; }

  // ---- Rumbo ----
  // Girar hasta el angulo absoluto target con velocidad angular maxima y aceleracion dadas.
  void turnTo(float target, float rate, float accel);
  bool turning() const { return !rotDone_; }
  // Correccion de rumbo de las paredes (grados) sumada al rumbo objetivo cuando no se gira.
  void setSteer(float deg) { steer_ = deg; }
  float headingTarget() const { return angleTarget_; }

  // Ciclo de control: calcula el duty de cada rueda.
  void update(const Sensors& s, float& dutyL, float& dutyR);
  // Parada inmediata: consignas = estado medido (sin tirones).
  void hold(const Sensors& s);

  // ---- Estado ----
  float position() const { return pos_; }       // recorrido medido (mm)
  float idealPosition() const { return ideal_; }
  float speed() const { return v_; }             // consigna (mm/s)
  float measuredSpeed() const { return vMeas_; } // medida filtrada
  float angle() const { return angle_; }         // rumbo fusionado (grados)
  // Factor de escala del giroscopio (aprendido con las paredes): grados reales por grado medido.
  void setGyroScale(float k) { gyroScale_ = k; }
  float gyroScale() const { return gyroScale_; }
  float idealAngle() const { return idealAngle_; }
  // Escala del modelo del motor aprendida en marcha (1 = los parametros; > 1 = motores mas debiles).
  float motorScale() const { return ffScale_; }
  // Velocidad de crucero que los motores dan de verdad (modelo aprendido, con margen para el rumbo).
  float topSpeed() const { return (0.9f * p_.maxDuty - p_.kS) / (ffScale_ * p_.kV); }
  void setMotorScale(float k) { ffScale_ = k; }
  float angularRate() const { return w_; }       // consigna (grados/s)
  float measuredRate() const { return wMeas_; }

 private:
  void measure(const Sensors& s);

  const Params& p_;
  // Medidas
  float encL0_ = 0, encR0_ = 0;
  float pos_ = 0, lastPos_ = 0, vMeas_ = 0;
  float gyroRaw_ = 0, gyroAtSample_ = 0, encAngleAtSample_ = 0, gyroScale_ = 1.0f;
  float dir_ = 1.0f, rotDir_ = 1.0f;
  float angle_ = 0, lastAngle_ = 0, wMeas_ = 0;
  // Avance
  float ideal_ = 0, v_ = 0, a_ = 0, end_ = 0, vmax_ = 0, vEnd_ = 0;
  bool done_ = true;
  float lastErrF_ = 0;
  // Rumbo
  float idealAngle_ = 0, w_ = 0, alpha_ = 0, angleTarget_ = 0, rate_ = 0, rotAccel_ = 0, steer_ = 0;
  bool rotDone_ = true;
  float lastErrR_ = 0;
  float iF_ = 0, iR_ = 0;
  float ffScale_ = 1.0f;
  float satF_ = 0;  // el ciclo anterior el avance iba con duty al maximo (+1 adelante, -1 atras)
};

}  // namespace raton
