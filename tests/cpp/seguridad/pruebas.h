void testOrquestadores() {
  using namespace raton;
  OrquestadorMovimiento mv;
  mv.empezarTramo(0);
  EntradaMovimiento in{};
  in.speed = 100;
  in.remaining = 300;
  in.front = -1;
  in.stopMm = 80;
  in.wallNearMm = 200;
  in.cellMm = 155;
  in.sideL = in.sideR = 30;
  for (uint32_t t = 0; t < 50; t += 10) {
    in.ms = t;
    CHECK(mv.decidir(in).seguridad == Seguridad::Ok);
  }
  // Pared delante: frena por el laser y avisa si es inesperada.
  in.front = 120;
  in.remaining = 600;
  DecisionMovimiento d = mv.decidir(in);
  CHECK(d.paredInesperada && d.remaining == 40);
  // Empujando: avanza 50 mm y el frontal no se acerca -> pegado.
  in.pos = 0;
  mv.decidir(in);
  in.pos = 50;
  CHECK(mv.decidir(in).seguridad == Seguridad::Pegado);
  // Roce sostenido > rubMs -> roce; al tercero, pegado.
  OrquestadorMovimiento r;
  r.empezarTramo(0);
  EntradaMovimiento ri = in;
  ri.front = -1;
  ri.sideL = 3;
  int roces = 0, pegado = 0;
  for (uint32_t t = 1; t < 2000; t += 20) {
    ri.ms = t;
    const Seguridad sg = r.decidir(ri).seguridad;
    roces += sg == Seguridad::Roce;
    if (sg == Seguridad::Pegado) {
      pegado = 1;
      break;
    }
  }
  CHECK(roces == 2 && pegado == 1);
}

