#include "modos/app.h"
#include "comunicacion/telemetry.h"
#include "config.h"

void App::modo2() {
  follower_.run(1, 0);
}
