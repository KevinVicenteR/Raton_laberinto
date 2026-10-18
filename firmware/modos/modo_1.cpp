#include "modos/app.h"
#include "comunicacion/telemetry.h"
#include "config.h"

void App::modo1() {
  follower_.run(-1, 0);
}
