#include "app/wall_follower.h"

#include "config.h"

void WallFollower::run(int side, float maxMm) {
  using Event = RobotHW::DriveEvent;
  const bool right = side < 0;
  const float start = robot_.travelledMm();
  while (!robot_.aborted() && (maxMm <= 0 || robot_.travelledMm() - start < maxMm)) {
    const raton::TrackGeometry& geo = robot_.geometry();
    const Event ev = robot_.drive(1e6f, side);
    if (ev == Event::SideOpen) {
      // Hueco en la pared que se sigue: llevar el eje de las ruedas al centro del hueco, girar y entrar.
      // El laser lateral mira por delante del eje y el hueco mide como el pasillo.
      const float half = 0.5f * geo.corridorMm();
      robot_.drive(fmaxf(geo.sideLookAhead() + half - cfg::OPEN_CONFIRM_MM, 0.0f), 0);
      log_.decide(side < 0 ? "hueco en la pared derecha -> entra" : "hueco en la pared izquierda -> entra");
      robot_.turnCorner(90.0f * side);
      // Entrar hasta que el laser lateral quede dentro del pasillo nuevo.
      robot_.drive(fmaxf(half - geo.sideLookAhead() + 15.0f, 15.0f), 0);
      robot_.ignoreSidesFor(half + geo.sideLookAhead());
    } else if (ev == Event::FrontWall && !maneuvers_.frontBlocked()) {
      maneuvers_.unstick();
    } else if (ev == Event::CrashLeft || ev == Event::CrashRight) {
      maneuvers_.nudge(ev == Event::CrashLeft);
    } else if (ev == Event::FrontWall) {
      const Walls w = robot_.readWalls();
      const bool followOpen = right ? !w.right : !w.left;
      const bool otherOpen = right ? !w.left : !w.right;
      // Retroceder lo justo para que las esquinas del raton no toquen la pared al girar.
      const int front = robot_.frontMm();
      const float clearance = front >= 0 ? static_cast<float>(geo.frontTurnMm() - front) : 0.0f;
      robot_.backUp(fmaxf(cfg::BACKUP_MM, clearance));
      if (followOpen) {
        log_.decide(right ? "pared delante, derecha libre -> gira derecha" : "pared delante, izquierda libre -> gira izquierda");
        robot_.turnCorner(90.0f * side);
      } else if (otherOpen) {
        log_.decide(right ? "pared delante y a la derecha -> gira izquierda" : "pared delante y a la izquierda -> gira derecha");
        robot_.turnCorner(-90.0f * side);
      } else {
        maneuvers_.deadEnd();
      }
      robot_.ignoreSidesFor(0.5f * geo.corridorMm() + geo.sideLookAhead());
    }
  }
}
