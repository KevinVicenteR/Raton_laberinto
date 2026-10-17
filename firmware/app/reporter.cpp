#include "app/reporter.h"

#include "comms/telemetry.h"

void Reporter::decide(const char* why) {
  char line[96];
  snprintf(line, sizeof(line), "DECIDE: %s | I%d F%d D%d", why, robot_.lasers().mm(Lasers::LEFT), robot_.frontMm(),
           robot_.lasers().mm(Lasers::RIGHT));
  telemetry::println(line);
  Serial.println(line);
}

void Reporter::event(const char* line) { telemetry::println(line); }
