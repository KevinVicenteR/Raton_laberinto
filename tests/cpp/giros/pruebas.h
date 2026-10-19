void testAngularProfile() {
  const float far = raton::angularTarget(1, 90, cfg::TURN_RATE_DPS, cfg::TURN_ACCEL_DPS2);
  const float near = raton::angularTarget(1, 2, cfg::TURN_RATE_DPS, cfg::TURN_ACCEL_DPS2);
  CHECK(far == cfg::TURN_RATE_DPS && near > 0 && near < far);
  CHECK(raton::angularTarget(-1, -2, cfg::TURN_RATE_DPS, cfg::TURN_ACCEL_DPS2) == -near);
  CHECK(raton::angularTarget(1, 1, cfg::TURN_RATE_DPS, cfg::TURN_ACCEL_DPS2) == 0);
}
