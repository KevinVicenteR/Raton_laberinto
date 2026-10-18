#include "modos/app.h"
#include "comunicacion/telemetry.h"
#include "config.h"

void App::modo4() {
  maze_.runCompetition();
}
