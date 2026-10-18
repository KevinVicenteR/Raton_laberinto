#include "modos/app.h"
#include "comunicacion/telemetry.h"
#include "config.h"

void App::modo3() {
  maze_.solveUnknown();
}
