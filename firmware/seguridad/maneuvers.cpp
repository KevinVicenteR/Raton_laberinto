#include "seguridad/maneuvers.h"

#include "config.h"

bool Maneuvers::frontBlocked() {
  const int f = robot_.frontMm();
  return f >= 0 && f < cfg::FRONT_WALL_MM;
}

void Maneuvers::unstick() {
  auto space = [](int mm) { return mm < 0 ? 10000 : mm; };
  const int left = space(robot_.sideMm(Lasers::LEFT));
  const int right = space(robot_.sideMm(Lasers::RIGHT));
  robot_.setState("DESATASCO");
  log_.decide(left > right ? "atascado con el frente libre -> atras y endereza a la izquierda"
                           : "atascado con el frente libre -> atras y endereza a la derecha");
  robot_.backUp(cfg::UNSTICK_BACK_MM);
  robot_.turnDegrees(left > right ? cfg::UNSTICK_DEG : -cfg::UNSTICK_DEG);
}

void Maneuvers::deadEnd() {
  log_.decide("callejon -> sale marcha atras buscando un hueco");
  const int side = robot_.backOut(3.0f * robot_.geometry().corridorMm());
  if (side != 0) {
    log_.decide(side > 0 ? "hueco a la izquierda al salir -> entra" : "hueco a la derecha al salir -> entra");
    robot_.turnCorner(90.0f * side);
  } else {
    log_.decide("sin hueco al salir -> media vuelta");
    robot_.turnDegrees(180.0f);
  }
}

void Maneuvers::nudge(bool rubbingLeft) {
  log_.decide(rubbingLeft ? "roza la pared izquierda -> endereza a la derecha"
                          : "roza la pared derecha -> endereza a la izquierda");
  robot_.backUp(cfg::NUDGE_BACK_MM);
  robot_.turnDegrees(rubbingLeft ? -cfg::NUDGE_DEG : cfg::NUDGE_DEG);
}
