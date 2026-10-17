#include "robot/command_handler.h"

#include <Arduino.h>

#include "comms/telemetry.h"
#include "config.h"
#include "protocol/frame.h"
#include "robot/robot_hw.h"

namespace {

float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

void reply(const char* line) {
  telemetry::println(line);
  Serial.println(line);
}

}  // namespace

void CommandHandler::poll() {
  while (Serial.available()) feed(Serial.read());
  for (int c = telemetry::read(); c >= 0; c = telemetry::read()) feed(c);
}

void CommandHandler::feed(int c) {
  if (c < 0) return;
  // 'x' suelta (fuera de una orden) para al momento, sin esperar al salto de linea.
  if ((c == 'x' || c == 'X') && cmdLen_ == 0) {
    robot_.abort();
    return;
  }
  if (c == '\n' || c == '\r') {
    if (cmdLen_ > 0) {
      cmd_[cmdLen_] = 0;
      onLine();
    }
    cmdLen_ = 0;
  } else if (cmdLen_ < static_cast<int>(sizeof(cmd_)) - 1) {
    cmd_[cmdLen_++] = static_cast<char>(c);
  }
}

void CommandHandler::onLine() {
  uint16_t seq = 0;
  char payload[sizeof(cmd_)];
  char ack[24];
  switch (raton::parseFrame(cmd_, seq, payload, sizeof(payload))) {
    case raton::FrameResult::Plain:
      run(cmd_);
      break;
    case raton::FrameResult::Ok:
      snprintf(ack, sizeof(ack), "ACK %u", static_cast<unsigned>(seq));
      telemetry::println(ack);
      if (seq != lastSeq_ || !seqValid_) {  // un reintento repetido no se ejecuta dos veces
        lastSeq_ = seq;
        seqValid_ = true;
        if ((payload[0] == 'x' || payload[0] == 'X') && payload[1] == 0) robot_.abort();
        else run(payload);
      }
      break;
    default:
      snprintf(ack, sizeof(ack), "NAK %u", static_cast<unsigned>(seq));
      telemetry::println(ack);
      break;
  }
}

void CommandHandler::run(const char* cmd) {
  Tuning& t = robot_.tuning();
  OperatorRequests& req = robot_.requests();
  raton::TrackGeometry& geo = robot_.geometry();
  Lasers& lasers = robot_.lasers();
  const float value = atof(cmd + 1);
  char line[160];
  switch (cmd[0]) {
    case 'v': t.speed = clampf(value, 30.0f, cfg::SPEED_RUN_SPEED); break;
    case 'k': t.kpWall = clampf(value, 0.0f, 1.0f); break;
    case 'j': t.kpSteer = clampf(value, 0.0f, 0.02f); break;
    case 'd': t.kdSteer = clampf(value, 0.0f, 5.0f); break;
    case 'p': t.steerMode = value >= 1 ? 1 : 0; break;
    case 'w': t.sideWall = static_cast<int>(clampf(value, 40.0f, 400.0f)); break;
    case 'z': geo.setDeadzone(static_cast<int>(clampf(value, 0.0f, 150.0f))); break;
    case 'r': lasers.setMaxMm(static_cast<int>(clampf(value, 50.0f, 2000.0f))); break;
    case 'g': lasers.setSignalLimit(clampf(value, 0.05f, 5.0f)); break;
    case 's':
      // s = arrancar en el modo elegido; s<n> = arrancar directamente en el modo n (si estaba en marcha,
      // para el modo actual y arranca el nuevo).
      if (value >= 1 && value <= 5) {
        req.mode = static_cast<int>(value);
        robot_.abort();
      }
      req.start = true;
      break;
    case 'm':
      // m<n>: elegir modo. Si estaba en marcha, se para y queda esperando ARRANCAR en el modo nuevo.
      if (value >= 1 && value <= 5) {
        req.mode = static_cast<int>(value);
        robot_.abort();
      }
      break;
    case 'G': {
      const char* comma = strchr(cmd, ',');
      if (comma) {
        req.goalX = atoi(cmd + 1);
        req.goalY = atoi(comma + 1);
      }
      break;
    }
    case 'n': {
      float v[4];
      const char* p = cmd + 1;
      int k = 0;
      while (k < 4 && *p) {
        v[k++] = atof(p);
        p = strchr(p, ',');
        if (!p) break;
        p++;
      }
      if (k == 4) {
        for (int i = 0; i < 4; ++i) t.weights[i] = clampf(v[i], -5.0f, 5.0f);
      }
      break;
    }
    case 'R':
      // Reiniciar la placa (como apagar y encender). Para los motores, deja salir el ACK y el aviso por
      // Bluetooth y reinicia; la web se vuelve a conectar sola.
      robot_.abort();
      robot_.coast();
      telemetry::println("REINICIO");
      telemetry::flush(500);  // sin escribir por USB: podria bloquear
      NVIC_SystemReset();
      break;
    case 'T':
      robot_.wheelTest();  // con el raton LEVANTADO
      break;
    case 'K': {
      // K<mm>: con el raton CENTRADO en un pasillo de ancho conocido, angulo real de los laterales y ese
      // pasillo. Sin numero: calibra el pasillo con el angulo que tenga.
      if (value >= 100 && value <= 400) {
        int rl, rr;
        if (robot_.calibrateSideAngle(value, rl, rr)) {
          snprintf(line, sizeof(line), "CALIB laterales: lecturas I%d D%d -> angulo %d grados (sen=%d/1000)", rl, rr,
                   static_cast<int>(asinf(geo.sideSin()) * 57.2958f), static_cast<int>(geo.sideSin() * 1000));
        } else {
          snprintf(line, sizeof(line), "CALIB fallo: hace falta pared a los dos lados (I%d D%d)", rl, rr);
        }
        reply(line);
      }
      int l, r, f;
      const bool ok = robot_.calibrateStill(l, r, f);
      snprintf(line, sizeof(line), "GEOM pasillo=%d celda=%d (quieto I%d F%d D%d%s)", static_cast<int>(geo.corridorMm()),
               static_cast<int>(geo.cellMm()), l, f, r, ok ? "" : " sin medir: hace falta pared a los dos lados");
      reply(line);
      return;
    }
    case 'c':
      // c<mm>: tamano de celda (pasillo + pared) a mano; el modo 3 ya no la cambia al medir.
      if (value >= 100 && value <= 400) {
        geo.setCorridor(value - geo.params().wallMm);
        geo.setManual(true);
      }
      break;
    case '?':
      break;
    default:
      reply("? s s<1-5> x R m<1-5> v p k j d r g w z n G c K T ?");
      return;
  }
  reportSettings();
}

void CommandHandler::reportSettings() {
  const Tuning& t = robot_.tuning();
  const OperatorRequests& req = robot_.requests();
  const raton::TrackGeometry& geo = robot_.geometry();
  Lasers& lasers = robot_.lasers();
  char line[160];
  snprintf(line, sizeof(line), "v=%d dir=%s k=%d/1000 j=%d/10000 d=%d/100 r=%d g=%d/100 w=%d z=%d meta=%d,%d",
           static_cast<int>(t.speed), t.steerMode ? "PID" : "cascada", static_cast<int>(t.kpWall * 1000),
           static_cast<int>(t.kpSteer * 10000), static_cast<int>(t.kdSteer * 100), lasers.maxMm(),
           static_cast<int>(lasers.signalLimit() * 100), t.sideWall, geo.deadzoneMm(), req.goalX, req.goalY);
  reply(line);
  snprintf(line, sizeof(line), "GEOM pasillo=%d celda=%d", static_cast<int>(geo.corridorMm()), static_cast<int>(geo.cellMm()));
  reply(line);
}
