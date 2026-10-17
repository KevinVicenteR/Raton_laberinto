// Lecturas parado y calibraciones del robot real: pasillo con los laterales, angulo real de los
// laterales, centrado en la salida, revision de motores y prueba de ruedas.
#include "comms/telemetry.h"
#include "config.h"
#include "robot/robot_hw.h"

namespace {

float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

// Mediana de v (se ordena en el sitio); -1 si hay menos de minCount valores.
int median(int* v, int n, int minCount) {
  if (n < minCount || n == 0) return -1;
  for (int a = 1; a < n; ++a)
    for (int b = a; b > 0 && v[b] < v[b - 1]; --b) {
      const int t = v[b];
      v[b] = v[b - 1];
      v[b - 1] = t;
    }
  return v[n / 2];
}

}  // namespace

int RobotHW::stableFrontMm() {
  // Varias lecturas parado. Mediana de las validas; con menos de 3 validas de 9 no se fia (-1), salvo
  // que la ultima valida fuera muy corta (pared pegada: por debajo de ~3 cm el VL53L0X no mide).
  int vals[9];
  int n = 0;
  for (int k = 0; k < 9; ++k) {
    lasers_.waitFresh(1);
    const int v = lasers_.mm(Lasers::FRONT);
    if (v >= 0) vals[n++] = v;
  }
  if (n >= 3) return median(vals, n, 3);
  if (lasers_.lastValidMm(Lasers::FRONT) < cfg::SIDE_CLOSE_MM + 20 && lasers_.lastValidAgeMs(Lasers::FRONT) < 1500) {
    return lasers_.lastValidMm(Lasers::FRONT);
  }
  return -1;
}

bool RobotHW::calibrateStill(int& left, int& right, int& front) {
  // Calibracion quieto (sin girar): con el raton en un pasillo, los laseres laterales en diagonal dan la
  // distancia de cada costado a su pared; pasillo = las dos + el ancho del cuerpo. Mediana de 15 lecturas.
  int ls[15], rs[15], nl = 0, nr = 0;
  for (int k = 0; k < 15; ++k) {
    lasers_.waitFresh(1);
    const int l = sideMm(Lasers::LEFT), r = sideMm(Lasers::RIGHT);
    if (l >= 0) ls[nl++] = l;
    if (r >= 0) rs[nr++] = r;
  }
  left = median(ls, nl, 8);
  right = median(rs, nr, 8);
  front = stableFrontMm();
  if (left < 0 || right < 0 || geom_.manual()) return false;
  const float corridor = left + right + cfg::BODY_WIDTH_MM;
  if (corridor + cfg::WALL_MM < 140.0f || corridor + cfg::WALL_MM > 400.0f) return false;
  geom_.setCorridor(corridor);
  return true;
}

bool RobotHW::calibrateSideAngle(float corridorMm, int& left, int& right) {
  // Con el raton CENTRADO en un pasillo de ancho conocido (medido con regla): hacia donde miran de
  // verdad los laseres laterales, y se fija ese pasillo.
  int ls[15], rs[15], nl = 0, nr = 0;
  for (int k = 0; k < 15; ++k) {
    lasers_.waitFresh(1);
    const int l = lasers_.mm(Lasers::LEFT), r = lasers_.mm(Lasers::RIGHT);
    if (l > 0) ls[nl++] = l;
    if (r > 0) rs[nr++] = r;
  }
  left = median(ls, nl, 8);
  right = median(rs, nr, 8);
  if (left <= 0 || right <= 0) return false;
  // Centrado: cada costado queda a (pasillo - cuerpo) / 2 de su pared.
  const float face = 0.5f * (corridorMm - cfg::BODY_WIDTH_MM);
  const float need = face + 0.5f * cfg::BODY_WIDTH_MM - cfg::SIDE_SENSOR_LATERAL_MM;
  geom_.setSideSin(clampf(need * 2.0f / (left + right), 0.3f, 0.98f));
  geom_.setCorridor(corridorMm);
  geom_.setManual(true);
  return true;
}

void RobotHW::alignToFront() {
  // Salida descentrada: si hay pared delante, se acerca o se aparta hasta la distancia de parada (el
  // raton queda centrado en la celda a lo largo). El descentrado lateral lo corrige el centrado al avanzar.
  const int f = stableFrontMm();
  const int stop = geom_.frontTurnMm();
  if (f < 0 || f > stop + static_cast<int>(0.75f * geom_.cellMm())) return;
  const float delta = clampf(static_cast<float>(f - stop), -60.0f, 60.0f);
  if (delta > 5.0f) {
    drive(delta, 0);
  } else if (delta < -5.0f) {
    backUp(-delta);
  }
  char line[48];
  snprintf(line, sizeof(line), "CENTRADO salida: frontal %d -> objetivo %d", f, stop);
  telemetry::println(line);
}

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
