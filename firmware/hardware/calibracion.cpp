#include "seguridad/robot_hw.h"
#include "comunicacion/telemetry.h"
#include "config.h"
#include "odometria/motion_progress.h"

void RobotHW::selfCheck(char* msg) {
  setState("REVISION");
  // Motores: la orden "girar a la izquierda" tiene que dar yaw positivo.
  gyro_.update();
  gyro_.resetYaw();
  encoders_.reset();
  uint32_t t0 = millis();
  motors_.set(-cfg::TURN_PWM_MAX, cfg::TURN_PWM_MAX);
  while (millis() - t0 < 250) gyro_.update();
  motors_.brake();
  t0 = millis();
  while (millis() - t0 < 200) gyro_.update();
  motors_.coast();
  const float yaw = gyro_.yaw();
  const bool motorsSwapped = yaw < -3.0f;
  if (motorsSwapped) motors_.setSwapped(true);
  // Distancia entre ruedas: lo que avanza una rueda menos la otra, entre el angulo girado.
  if (!motorsSwapped && yaw > 10.0f) {
    const float diff = (encoders_.right() - encoders_.left()) * cfg::MM_PER_COUNT;
    const float tw = diff / (yaw / 57.2958f);
    if (tw > 40.0f && tw < 160.0f) trackWidth_ = tw;
  }
  rotate(-yaw);  // volver al rumbo inicial (ya con los motores bien)
  // Los laseres no se comprueban girando: si estan cruzados, se ve en la web (I/D al reves).
  snprintf(msg, 22, "Mot %s Las ok", motorsSwapped ? "CRUZ" : "ok");
}

void RobotHW::wheelTest() {
  // Cada rueda sola hacia delante y hacia atras; manda lo que cuenta cada encoder y lo que gira el
  // giroscopio. Si una rueda cuenta mucho menos que la otra con la misma potencia, patina, roza o su
  // encoder pierde cuentas.
  struct Step {
    float l, r;
    const char* name;
  } steps[] = {{0.35f, 0, "izq adelante"}, {-0.35f, 0, "izq atras"}, {0, 0.35f, "der adelante"},
               {0, -0.35f, "der atras"}, {0.35f, 0.35f, "las dos adelante"}};
  for (const Step& st : steps) {
    encoders_.reset();
    gyro_.update();
    gyro_.resetYaw();
    motors_.set(st.l, st.r);
    const uint32_t t0 = millis();
    while (millis() - t0 < 600) gyro_.update();
    motors_.brake();
    const uint32_t t1 = millis();
    while (millis() - t1 < 200) gyro_.update();
    motors_.coast();
    char line[80];
    snprintf(line, sizeof(line), "PRUEBA %s: izq=%ld der=%ld giro=%d", st.name, static_cast<long>(encoders_.left()),
             static_cast<long>(encoders_.right()), static_cast<int>(gyro_.yaw()));
    telemetry::println(line);
    Serial.println(line);
  }
}
