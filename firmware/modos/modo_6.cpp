#include "modos/app.h"
#include "comunicacion/telemetry.h"
#include "config.h"

void App::modo6() {
  maze_.learnTrack();
}
