void testTrackGeometry() {
  raton::TrackGeometry g(raton::GeometryParams(), 140.0f, 155.0f, 31.0f, 50);
  CHECK(g.frontTurnMm() == 80);    // 70 - 40 + 50
  CHECK(g.cellFrontWall() == 140);
  CHECK(g.cellSideWall() == 58);   // 0.42 x 140
  CHECK(g.sideFromRay(-1) == -1);
  CHECK(g.sideFromRay(60) == 21);  // 60 x sen(31) + 30 - 40
  CHECK(g.sideFromRay(5) == 0);
  g.setCorridor(192);
  CHECK(g.corridorMm() == 192 && g.cellMm() == 204);
  g.setCorridor(1000);
  CHECK(g.corridorMm() == 400);
  g.blendCorridor(400);
  CHECK(g.cellMm() == 412);  // el ajuste en marcha no toca la celda
  g.setCorridor(100);
  g.setDeadzone(0);
  CHECK(g.frontTurnMm() == 20);  // 50 - 40 + 0 queda por debajo de punto muerto + margen  // nunca menos que punto muerto + margen
}


void testWallMedian() {
  int samples[] = {31, 30, 250, 29, 32};
  CHECK(raton::medianOf(samples, 5) == 31);  // una lectura fantasma no crea hueco
  CHECK(raton::medianOf(samples, 0) == -1);
  raton::CellWindow window;
  CHECK(window.nWinL == 0 && window.nWinR == 0 && !window.windowSeen);
}
