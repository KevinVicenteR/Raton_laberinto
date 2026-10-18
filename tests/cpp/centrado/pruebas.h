void testSteering() {
  const raton::SteeringParams p{cfg::LATERAL_DEADBAND_MM, cfg::KD_YAW, cfg::KP_YAW,
      cfg::WALL_RATE_DAMP, cfg::MAX_WALL_CORRECTION_DEG, cfg::YAW_TARGET_SMOOTH, cfg::MAX_STEER};
  float reference = 0, target = 0;
  const auto step = [&](bool walls, float offset, float heading, int mode) {
    return raton::steering(100, 0.005f, walls, offset, 0, 1, mode, cfg::KP_STEER,
        cfg::KD_STEER, cfg::KP_WALL, heading, 0, reference, target, p);
  };
  CHECK(step(true, 1, 0, 0) == 0);  // banda muerta, sin deriva
  CHECK(step(true, 30, 0, 0) > 0);
  reference = target = 0;
  CHECK(step(true, -30, 0, 0) < 0);
  reference = target = 0;
  CHECK(step(false, 0, 5, 1) < 0);  // sin paredes, corrige rumbo
  CHECK(std::fabs(step(true, 10000, 0, 1)) <= cfg::MAX_STEER);
}
