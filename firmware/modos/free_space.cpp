#include "modos/free_space.h"

#include "comunicacion/telemetry.h"
#include "config.h"

namespace {
int space(int mm) { return mm < 0 ? 10000 : mm; }  // sin lectura = muy lejos
}  // namespace

int FreeSpaceNavigator::sideSpace(Lasers::Id id) {
  Lasers& lasers = robot_.lasers();
  const int mm = robot_.sideMm(id);
  if (mm < 0 && lasers.lastValidMm(id) < cfg::SIDE_CLOSE_MM && lasers.lastValidAgeMs(id) < 300) {
    return 0;  // sin lectura justo tras una muy corta: pegado a la pared
  }
  return space(mm);
}

bool FreeSpaceNavigator::run() {
  using Event = RobotHW::DriveEvent;
  float lastTurnAt = robot_.travelledMm();
  int turnsInPlace = 0;  // giros seguidos sin avanzar: esta dando vueltas en el mismo sitio
  int stuck = 0;         // atascos seguidos con el frente libre
  while (!robot_.aborted()) {
    if (robot_.goalReached()) return true;
    if (turnsInPlace >= cfg::LOOP_TURNS) {
      // Atrapado girando en un cruce: seguir la pared derecha un tramo para salir de ahi.
      telemetry::println("BUCLE: sigue la pared un tramo");
      follower_.run(-1, cfg::LOOP_ESCAPE_MM);
      turnsInPlace = 0;
      lastTurnAt = robot_.travelledMm();
      continue;
    }
    const Event ev = robot_.drive(1e6f, RobotHW::kWatchBoth);
    if (ev == Event::CrashLeft || ev == Event::CrashRight) {
      maneuvers_.nudge(ev == Event::CrashLeft);
      continue;
    }
    if (ev != Event::SideOpen && ev != Event::FrontWall) continue;
    if (ev == Event::FrontWall && !maneuvers_.frontBlocked()) {
      // Atasco con el frente libre: desatascar; a la tercera seguida, salir marcha atras.
      if (++stuck < 3) {
        maneuvers_.unstick();
      } else {
        stuck = 0;
        maneuvers_.deadEnd();
      }
      continue;
    }
    stuck = 0;

    const raton::TrackGeometry& geo = robot_.geometry();
    const float half = 0.5f * geo.corridorMm();
    if (ev == Event::SideOpen) {
      // El laser lateral va por delante del eje: avanzar hasta que el eje quede en el centro del hueco.
      robot_.drive(fmaxf(geo.sideLookAhead() + half - cfg::OPEN_CONFIRM_MM, 0.0f), 0);
    }

    // Parado en el cruce: leer los tres laseres con medidas nuevas.
    robot_.setState("CRUCE");
    robot_.lasers().waitFresh(3);
    const int left = sideSpace(Lasers::LEFT);
    const int right = sideSpace(Lasers::RIGHT);
    const int frontRaw = robot_.frontMm();
    const int front = space(frontRaw);
    const int sideWall = robot_.tuning().sideWall;
    const bool leftOpen = left > sideWall;
    const bool rightOpen = right > sideWall;
    const bool frontOpen = front > cfg::FRONT_WALL_MM;
    char line[48];
    snprintf(line, sizeof(line), "CRUCE izq%d del%d der%d", left, front, right);
    telemetry::println(line);

    // Puntuacion lineal por direccion: espacio libre, novedad, coste de girar, sesgo recto, direccion
    // hacia la meta (si la hay) y salidas ya usadas de este cruce (memoria Tremaux).
    const float* w = robot_.tuning().weights;
    auto score = [&](int mm, float turnDeg) {
      const float freeN = (mm > 300 ? 300 : mm) / 300.0f;
      const float novel = 1.0f / (1.0f + robot_.visitsToward(turnDeg));
      return w[0] * freeN + w[1] * novel - w[2] * fabsf(turnDeg) / 90.0f + (turnDeg == 0.0f ? w[3] : 0.0f) +
             cfg::W_GOAL * robot_.goalAlignment(turnDeg) - cfg::W_EXIT * robot_.exitsToward(turnDeg);
    };
    const float sF = frontOpen ? score(front, 0) : -99, sL = leftOpen ? score(left, 90) : -99,
                sR = rightOpen ? score(right, -90) : -99;
    int turn = 180;  // grados: +90 izquierda, -90 derecha, 0 recto, 180 media vuelta (callejon)
    float best = -50;
    if (sF > best) { best = sF; turn = 0; }
    if (sL > best) { best = sL; turn = 90; }
    if (sR > best) { best = sR; turn = -90; }
    char why[80];
    snprintf(why, sizeof(why), "cruce izq %d recto %d der %d (x100) -> %s", static_cast<int>(sL * 100),
             static_cast<int>(sF * 100), static_cast<int>(sR * 100),
             turn == 0 ? "recto" : turn == 90 ? "izquierda" : turn == -90 ? "derecha" : "callejon");

    if (turn != 180) robot_.markExit(static_cast<float>(turn));
    if (turn == 0) {
      log_.decide(why);
      // Recto: pasar el hueco sin volver a detectarlo.
      robot_.drive(half + geo.sideLookAhead(), 0);
      robot_.ignoreSidesFor(half);
      continue;
    }
    if (frontRaw >= 0) {
      const float clearance = static_cast<float>(geo.frontTurnMm() - frontRaw);
      if (clearance > 0) robot_.backUp(clearance);
    }
    turnsInPlace = robot_.travelledMm() - lastTurnAt < cfg::LOOP_MIN_TRAVEL_MM ? turnsInPlace + 1 : 1;
    lastTurnAt = robot_.travelledMm();
    if (turn == 180) {
      maneuvers_.deadEnd();
    } else {
      log_.decide(why);
      robot_.turnCorner(static_cast<float>(turn));
      // Entrar en el pasillo nuevo hasta que los laterales vean sus paredes.
      robot_.drive(fmaxf(half - geo.sideLookAhead() + 15.0f, 15.0f), 0);
    }
    robot_.ignoreSidesFor(half + geo.sideLookAhead());
  }
  return robot_.goalReached();
}
