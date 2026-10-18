void testAdaptiveSpeed() {
  {
    raton::SpeedPolicy p;
    CHECK(raton::AdaptiveSpeed::target(p, 300, 1, 0, true, -1, false) == 300);
    CHECK(raton::AdaptiveSpeed::target(p, 300, 1, 0, false, -1, false) == 270);
    CHECK(std::fabs(raton::AdaptiveSpeed::target(p, 300, 0, 0, true, -1, false) - 180) < 0.01f);
    CHECK(std::fabs(raton::AdaptiveSpeed::target(p, 300, 1, 5, true, -1, false) - 180) < 0.01f);
    CHECK(std::fabs(raton::AdaptiveSpeed::target(p, 300, 1, 0, true, 0, false) - 180) < 0.01f);
    raton::AdaptiveSpeed speed;
    speed.caution(100);
    CHECK(std::fabs(speed.step(p, 300, 1, 0, true, -1, 50, 0.005f, 1000) - 180) < 0.01f);
    CHECK(speed.cautionLeft() == 50);
    CHECK(std::fabs(speed.step(p, 300, 1, 0, true, -1, 51, 0.005f, 1000) - 185) < 0.01f);
    CHECK(speed.cautionLeft() == 0);
    // La bajada tambien es gradual y un retroceso no consume la distancia cauta.
    speed.caution(80);
    CHECK(std::fabs(speed.step(p, 300, 1, 0, true, -1, -20, 0.005f, 1000) - 180) < 0.01f);
    CHECK(speed.cautionLeft() == 80);
  }
}

void testSpeedPI() {
  const raton::VelocityParams p{cfg::PWM_MIN_RUN, cfg::KFF_SPEED, cfg::KP_SPEED,
      cfg::KI_SPEED, cfg::PWM_MIN_MOVE, cfg::PWM_MAX, cfg::MAX_STEER};
  float integral = 0;
  const float launch = raton::speedOutput(100, 0, integral, 0.005f, p);
  CHECK(launch >= cfg::PWM_MIN_MOVE);
  integral = 0;
  // Debe poder quitar potencia por debajo del mantenimiento al exceder la consigna.
  const float braking = raton::speedOutput(40, 300, integral, 0.005f, p);
  CHECK(braking < cfg::PWM_MIN_RUN && braking >= 0);
  for (int i = 0; i < 1000; ++i) raton::speedOutput(700, 0, integral, 0.005f, p);
  CHECK(integral == 50);
  CHECK(raton::speedOutput(0, 0, integral, 0.005f, p) == 0);
}
