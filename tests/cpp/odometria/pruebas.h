void testMotionProgress() {
  // Tramo de 8 celdas interrumpido tras 2: el residuo no puede ser -6 celdas.
  const auto partial = raton::motionProgress(320, 0, 155, 8, false);
  CHECK(partial.cells == 2 && partial.offset == 10);
  const auto reverse = raton::motionProgress(300, 10, 155, 8, true);
  CHECK(reverse.cells == 2 && reverse.offset == 20);
  const auto stopped = raton::motionProgress(0, 30, 155, 1, true);
  CHECK(stopped.cells == 0 && stopped.offset == 30);
  const auto overshoot = raton::motionProgress(330, 0, 155, 1, false);
  CHECK(overshoot.cells == 1 && overshoot.offset == 175);
}

