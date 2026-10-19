#include "modos/app.h"
#include "comunicacion/telemetry.h"
#include "config.h"

void App::modo5() {
      // Pista desconocida: con meta (G x,y desde la web) va hacia ella; sin meta, explora. La meta en
      // odometria: el arranque es el eje centrado en la salida (START_X, START_Y) mirando al norte.
      const OperatorRequests& req = robot_.requests();
      if (req.goalX >= 0) {
        const float cell = robot_.geometry().cellMm();
        robot_.setGoal((req.goalX - cfg::START_X) * cell, (req.goalY - cfg::START_Y) * cell);
      }
      if (freeSpace_.run()) {
        robot_.setState("META");
        telemetry::println("META");
      }
}
