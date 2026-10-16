#include "robot_hw.h"

#include "config.h"
#include "motion_progress.h"
#include "risk_model.h"
#include "telemetry.h"

namespace {

float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

}  // namespace

RobotHW::RobotHW(Motors& motors, Encoders& encoders, Lasers& lasers, Gyro& gyro, Display& display)
    : motors_(motors), encoders_(encoders), lasers_(lasers), gyro_(gyro), display_(display) {}

void RobotHW::waitPeriod(uint32_t& next) {
  const uint32_t now = micros();
  if (lastPeriodUs_ != 0 && now - lastPeriodUs_ > maxLagUs_) maxLagUs_ = now - lastPeriodUs_;
  // Si el ciclo se retraso mucho, no intentar recuperar ciclos perdidos de golpe.
  if (static_cast<int32_t>(now - next) > static_cast<int32_t>(4 * cfg::CONTROL_PERIOD_US)) next = now;
  while (static_cast<int32_t>(micros() - next) < 0) {
    lasers_.update();
    gyro_.update();
  }
  telemetry::poll();
  handleInput();
  updateOdometry();
  sampleRiskHistory();
  sendTelemetry();
  lastPeriodUs_ = micros();
  next += cfg::CONTROL_PERIOD_US;
}

raton::SensorObservation RobotHW::sensorObservation(int i) const {
  const auto id=static_cast<Lasers::Id>(i);
  return {static_cast<float>(lasers_.mm(id)),lasers_.confidence(id),
          static_cast<float>(lasers_.lastValidAgeMs(id)),lasers_.mm(id)>=0};
}

void RobotHW::sampleRiskHistory() {
  const uint32_t count=lasers_.sampleCount(Lasers::LEFT)+lasers_.sampleCount(Lasers::FRONT)+
                       lasers_.sampleCount(Lasers::RIGHT);
  if(count==riskSampleCount_) return;
  riskSampleCount_=count;
  riskHistory_[0]=riskHistory_[1];riskHistory_[1]=riskHistory_[2];
  for(int i=0;i<3;++i)riskHistory_[2].sensor[i]=sensorObservation(i);
}

void RobotHW::controlNetStep(float remaining, bool reverse) {
  if (!controlNetOn()) {
    ctrlYaw_ = 0;
    ctrlScale_ = 1;
    return;
  }
  if (ctrlSampleCount_ == riskSampleCount_) return;  // solo con lecturas nuevas (como en el simulador)
  ctrlSampleCount_ = riskSampleCount_;
  float x[raton::kControlInputs], out[2];
  raton::controlFeatures(riskHistory_, lastSpeed_, gyro_.rate(), gridOn_ ? gridError() : 0, remaining,
                         wallTarget_, reverse, x);
  if (raton::mlpForward(raton::kControlModel, x, out)) {
    raton::controlOutputs(out, ctrlYaw_, ctrlScale_);
  } else {
    ctrlYaw_ = 0;  // entrada o salida invalida: control normal
    ctrlScale_ = 1;
  }
  if (millis() - lastCtrlTelemetry_ >= 200) {
    lastCtrlTelemetry_ = millis();
    char line[48];
    snprintf(line, sizeof(line), "CTRL dyaw=%d vscale=%d", static_cast<int>(ctrlYaw_ * 10),
             static_cast<int>(ctrlScale_ * 100));
    telemetry::println(line);
  }
}

bool RobotHW::actionAllowed(float degrees) {
  if(!raton::kRiskModelTrained || fabsf(degrees)==180) return true;
  float x[43];
  const int action=degrees==0?0:(degrees>0?1:2);
  raton::riskFeatures(riskHistory_,lastSpeed_,gyro_.rate(),gridOn_?gridError():0,
                      pendingAdvance_,action,x);
  const float risk=raton::predictRisk(raton::kRiskModel,x);
  char line[96];
  snprintf(line,sizeof(line),"RISK action=%d per_mille=%d enabled=%d",action,
           std::isfinite(risk)?static_cast<int>(risk*1000):-1,raton::kRiskModelEnabled);
  telemetry::println(line);
  return !raton::kRiskModelEnabled || !std::isfinite(risk) || risk<=raton::kRiskThreshold;
}

void RobotHW::recordMotion(const char* phase,float requested,const raton::MotionResult& result) {
  lastMotion_=result;
  char line[384];
  snprintf(line,sizeof(line),"MOTION t=%lu phase=%s req=%d actual=%d error_cdeg=%d completed=%d reason=%d enc=%ld,%ld raw=%d,%d,%d filtered=%d,%d,%d age=%lu,%lu,%lu conf=%d,%d,%d",
           static_cast<unsigned long>(millis()),phase,static_cast<int>(requested),
           static_cast<int>(result.actualMm),static_cast<int>(result.angularErrorDeg*100),
           result.completed,static_cast<int>(result.reason),encoders_.leftTotal(),encoders_.rightTotal(),
           lasers_.acquiredMm(Lasers::LEFT),lasers_.acquiredMm(Lasers::FRONT),lasers_.acquiredMm(Lasers::RIGHT),
           lasers_.mm(Lasers::LEFT),lasers_.mm(Lasers::FRONT),lasers_.mm(Lasers::RIGHT),
           static_cast<unsigned long>(lasers_.lastValidAgeMs(Lasers::LEFT)),
           static_cast<unsigned long>(lasers_.lastValidAgeMs(Lasers::FRONT)),
           static_cast<unsigned long>(lasers_.lastValidAgeMs(Lasers::RIGHT)),
           static_cast<int>(100*lasers_.confidence(Lasers::LEFT)),static_cast<int>(100*lasers_.confidence(Lasers::FRONT)),
           static_cast<int>(100*lasers_.confidence(Lasers::RIGHT)));
  telemetry::println(line);
}

void RobotHW::resetOdometry() {
  odoX_ = 0;
  odoY_ = 0;
  odoYaw0_ = gyro_.totalYaw();
  odoLastA_ = 0;
  memset(visits_, 0, sizeof(visits_));
  memset(exits_, 0, sizeof(exits_));
  pendingAdvance_ = 0;
  for(auto& frame:riskHistory_)frame=raton::RiskFrame{};
  riskSampleCount_=0;
  cellX_ = cellY_ = -1;
  odoL_ = encoders_.leftTotal();
  odoR_ = encoders_.rightTotal();
}

void RobotHW::updateOdometry() {
  const long l = encoders_.leftTotal();
  const long r = encoders_.rightTotal();
  const float ds = 0.5f * ((l - odoL_) + (r - odoR_)) * cfg::MM_PER_COUNT;
  odoL_ = l;
  odoR_ = r;
  odoDist_ += ds;
  for (int i = 0; i < 3; ++i) {
    if (lasers_.mm(static_cast<Lasers::Id>(i)) >= 0) lastValidDist_[i] = odoDist_;
  }
  // El rumbo sale del giroscopio (no patina); 0 = mirando hacia +Y al empezar. Se integra con el
  // rumbo medio del paso para que los giros en marcha no deformen el mapa.
  const float a = (gyro_.totalYaw() - odoYaw0_) / 57.2958f;
  const float am = 0.5f * (a + odoLastA_);
  odoLastA_ = a;
  odoX_ -= ds * sinf(am);
  odoY_ += ds * cosf(am);
  const int cx = cellOf(odoX_), cy = cellOf(odoY_ + 0.5f * corridorMm_);
  if (cx != cellX_ || cy != cellY_) {
    cellX_ = cx;
    cellY_ = cy;
    if (cx >= 0 && cy >= 0 && cx < kGrid && cy < kGrid && visits_[cx][cy] < 255) visits_[cx][cy]++;
  }
}

int RobotHW::cellOf(float v) const {
  const float pitch = corridorMm_ + 15.0f;  // pasillo + pared
  return static_cast<int>(floorf(v / pitch + 0.5f)) + kGrid / 2;
}

int RobotHW::worldDir(float turnDeg) const {
  const float h = (gyro_.totalYaw() - odoYaw0_) + turnDeg;
  return ((static_cast<int>(lroundf(h / 90.0f)) % 4) + 4) % 4;
}

float RobotHW::goalAlignment(float turnDeg) const {
  if (!goalSet_) return 0.0f;
  const float gx = goalX_mm_ - odoX_, gy = goalY_mm_ - odoY_;
  const float len = hypotf(gx, gy);
  if (len < 1.0f) return 1.0f;
  const int d = worldDir(turnDeg);
  const float dx[4] = {0, -1, 0, 1}, dy[4] = {1, 0, -1, 0};
  return (dx[d] * gx + dy[d] * gy) / len;
}

int RobotHW::exitsToward(float turnDeg) const {
  if (cellX_ < 0 || cellX_ >= kGrid || cellY_ < 0 || cellY_ >= kGrid) return 0;
  return exits_[cellX_][cellY_][worldDir(turnDeg)];
}

void RobotHW::markExit(float turnDeg) {
  if (cellX_ < 0 || cellX_ >= kGrid || cellY_ < 0 || cellY_ >= kGrid) return;
  uint8_t& e = exits_[cellX_][cellY_][worldDir(turnDeg)];
  if (e < 255) e++;
}

int RobotHW::visitsToward(float turnDeg) const {
  if (cellX_ < 0) return 0;
  // Direccion en la pista: 0 = norte (+y), 1 = oeste, 2 = sur, 3 = este (angulos positivos a la izquierda).
  const float h = (gyro_.totalYaw() - odoYaw0_) + turnDeg;
  const int d = ((static_cast<int>(lroundf(h / 90.0f)) % 4) + 4) % 4;
  const int dx[4] = {0, -1, 0, 1}, dy[4] = {1, 0, -1, 0};
  const int x = cellX_ + dx[d], y = cellY_ + dy[d];
  if (x < 0 || y < 0 || x >= kGrid || y >= kGrid) return 0;
  return visits_[x][y];
}

void RobotHW::idle(uint32_t ms) {
  const uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    lasers_.update();
    gyro_.update(true);
    telemetry::poll();
    handleInput();
    updateOdometry();
    sendTelemetry();
  }
}

void RobotHW::stopMotors() {
  lastPeriodUs_ = 0;  // la pausa entre maniobras no cuenta como retraso del control
  motors_.brake();
  delay(60);
  motors_.coast();
  lastSpeed_ = 0;
  idle(cfg::SETTLE_MS);
}

void RobotHW::resetHeading() {
  gyro_.update();
  gyro_.resetYaw();
  heading_ = 0;
  headingBase_ = 0;
  lastGyroYaw_ = 0;
}

void RobotHW::resetHeadingToGrid() {
  resetHeading();
  // Con cuadricula, el rumbo cero es el de la pista, no hacia donde mira ahora.
  if (gridOn_) heading_ = headingBase_ = -gridError();
}

float RobotHW::gridError() const {
  // Cuanto esta girado respecto al rumbo de la cuadricula mas cercano (multiplo de 90).
  float e = fmodf(gyro_.totalYaw() - gridYaw_, 90.0f);
  if (e > 45.0f) e -= 90.0f;
  if (e < -45.0f) e += 90.0f;
  return e;
}

void RobotHW::updateHeading() {
  // Filtro complementario: el giroscopio manda a corto plazo; los encoders (sin deriva, pero con
  // patinazos) corrigen poco a poco. Requiere encoders_.reset() junto con resetHeading().
  const float g = gyro_.yaw();
  heading_ += g - lastGyroYaw_;
  lastGyroYaw_ = g;
  const float diff = (encoders_.right() - encoders_.left()) * cfg::MM_PER_COUNT;
  const float encYaw = headingBase_ + diff / trackWidth_ * 57.2958f;
  heading_ += cfg::ENC_YAW_WEIGHT * (encYaw - heading_);
}

void RobotHW::handleInput() {
  auto feed = [this](int c) {
    if (c < 0) return;
    if (c == 'x' || c == 'X') {
      aborted_ = true;
      cmdLen_ = 0;
      return;
    }
    if (c == '\n' || c == '\r') {
      if (cmdLen_ > 0) {
        cmd_[cmdLen_] = 0;
        runCommand(cmd_);
      }
      cmdLen_ = 0;
    } else if (cmdLen_ < static_cast<int>(sizeof(cmd_)) - 1) {
      cmd_[cmdLen_++] = static_cast<char>(c);
    }
  };
  while (Serial.available()) feed(Serial.read());
  for (int c = telemetry::read(); c >= 0; c = telemetry::read()) feed(c);
}

void RobotHW::runCommand(const char* cmd) {
  char reply[160];
  const float value = atof(cmd + 1);
  switch (cmd[0]) {
    case 'v':
      speed_ = clampf(value, 30.0f, cfg::SPEED_RUN_SPEED);
      break;
    case 'k':
      kpWall_ = clampf(value, 0.0f, 1.0f);
      break;
    case 'j':
      kpSteer_ = clampf(value, 0.0f, 0.02f);
      break;
    case 'd':
      kdSteer_ = clampf(value, 0.0f, 5.0f);
      break;
    case 'p':
      steerMode_ = value >= 1 ? 1 : 0;
      break;
    case 's':
      startRequested_ = true;
      break;
    case 'G': {
      // G<x>,<y>: celda meta para el modo laberinto
      const char* comma = strchr(cmd, ',');
      if (comma) {
        goalX_ = atoi(cmd + 1);
        goalY_ = atoi(comma + 1);
      }
      break;
    }
    case 'n': {
      // n<espacio>,<novedad>,<giro>,<sesgo>
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
        for (int i = 0; i < 4; ++i) w_[i] = clampf(v[i], -5.0f, 5.0f);
      }
      break;
    }
    case 'w':
      sideWall_ = static_cast<int>(clampf(value, 40.0f, 400.0f));
      break;
    case 'z':
      deadzone_ = static_cast<int>(clampf(value, 0.0f, 150.0f));
      break;
    case 'r':
      lasers_.setMaxMm(static_cast<int>(clampf(value, 50.0f, 2000.0f)));
      break;
    case 'g':
      lasers_.setSignalLimit(clampf(value, 0.05f, 5.0f));
      break;
    case 'm':
      if (value >= 1 && value <= 5) requestedMode_ = static_cast<int>(value);
      break;
    case 'N': {
      // N<c|l><0|1>: encender o apagar la red de control (c) o la de logica (l)
      const bool on = cmd[2] == '1';
      const bool trained = cmd[1] == 'c' ? raton::kControlModelTrained : raton::kLogicModelTrained;
      if (cmd[1] == 'c') ctrlNetOn_ = on;
      else if (cmd[1] == 'l') logicNetOn_ = on;
      if (on && !trained) telemetry::println("NET aviso: esa red no esta entrenada, no hara nada");
      break;
    }
    case '?':
      break;
    default:
      snprintf(reply, sizeof(reply), "? s x m<1-5> v p k j d r g w z n G N<c|l><0|1> ?");
      telemetry::println(reply);
      Serial.println(reply);
      return;
  }
  snprintf(reply, sizeof(reply), "v=%d dir=%s k=%d/1000 j=%d/10000 d=%d/100 r=%d g=%d/100 w=%d z=%d meta=%d,%d",
           static_cast<int>(speed_), steerMode_ ? "PID" : "cascada", static_cast<int>(kpWall_ * 1000),
           static_cast<int>(kpSteer_ * 10000), static_cast<int>(kdSteer_ * 100), lasers_.maxMm(), static_cast<int>(lasers_.signalLimit() * 100),
           sideWall_, deadzone_, goalX_, goalY_);
  telemetry::println(reply);
  Serial.println(reply);
  snprintf(reply, sizeof(reply), "NET control=%d control_trained=%d logic=%d logic_trained=%d", ctrlNetOn_ ? 1 : 0,
           raton::kControlModelTrained ? 1 : 0, logicNetOn_ ? 1 : 0, raton::kLogicModelTrained ? 1 : 0);
  telemetry::println(reply);
  Serial.println(reply);
}

void RobotHW::sendTelemetry() {
  if (millis() - lastTelemetry_ < cfg::TELEMETRY_PERIOD_MS) return;
  lastTelemetry_ = millis();
  if(millis()-lastObservationTelemetry_>=250) {
    lastObservationTelemetry_=millis();
  char sample[256];
  snprintf(sample,sizeof(sample),"OBS t=%lu raw=%d,%d,%d filtered=%d,%d,%d age=%lu,%lu,%lu conf=%d,%d,%d enc=%ld,%ld yaw=%d phase=%s drops=%lu",
    static_cast<unsigned long>(millis()),lasers_.acquiredMm(Lasers::LEFT),lasers_.acquiredMm(Lasers::FRONT),lasers_.acquiredMm(Lasers::RIGHT),
    lasers_.mm(Lasers::LEFT),lasers_.mm(Lasers::FRONT),lasers_.mm(Lasers::RIGHT),
    static_cast<unsigned long>(lasers_.lastValidAgeMs(Lasers::LEFT)),static_cast<unsigned long>(lasers_.lastValidAgeMs(Lasers::FRONT)),
    static_cast<unsigned long>(lasers_.lastValidAgeMs(Lasers::RIGHT)),static_cast<int>(100*lasers_.confidence(Lasers::LEFT)),
    static_cast<int>(100*lasers_.confidence(Lasers::FRONT)),static_cast<int>(100*lasers_.confidence(Lasers::RIGHT)),
    encoders_.leftTotal(),encoders_.rightTotal(),static_cast<int>(gyro_.totalYaw()*100),state_,static_cast<unsigned long>(telemetry::droppedLines()));
  telemetry::println(sample);
  }
  char line[160];
  const int a = static_cast<int>(gyro_.totalYaw() - odoYaw0_);
  // q = lecturas SIN filtrar (para el mapa: el filtro mantiene valores viejos cuando el laser no mide).
  snprintf(line, sizeof(line), "%s I%d F%d D%d v%d r%d p%d x%d y%d a%d t%d q%d,%d,%d c%d,%d,%d g%d", state_,
           lasers_.mm(Lasers::LEFT), lasers_.mm(Lasers::FRONT), lasers_.mm(Lasers::RIGHT),
           static_cast<int>(lastSpeed_), static_cast<int>(heading_), static_cast<int>(corridorMm_),
           static_cast<int>(odoX_), static_cast<int>(odoY_), a, static_cast<int>(maxLagUs_ / 1000),
           lasers_.rawMm(Lasers::LEFT), lasers_.rawMm(Lasers::FRONT), lasers_.rawMm(Lasers::RIGHT),
           static_cast<int>(lasers_.confidence(Lasers::LEFT) * 100), static_cast<int>(lasers_.confidence(Lasers::FRONT) * 100),
           static_cast<int>(lasers_.confidence(Lasers::RIGHT) * 100), static_cast<int>(mapConfidence() * 100));
  maxLagUs_ = 0;
  telemetry::println(line);
}

int RobotHW::frontMm() {
  const int f = lasers_.mm(Lasers::FRONT);
  if (f >= 0) {
    frontMemMm_ = f;
    frontMemDist_ = odoDist_;
    return f;
  }
  if (frontMemMm_ >= 0 && frontMemMm_ < cfg::FRONT_MEMORY_MM) {
    const float predicted = frontMemMm_ - (odoDist_ - frontMemDist_);
    if (predicted < cfg::FRONT_MEMORY_MM) return predicted > 0 ? static_cast<int>(predicted) : 0;
  }
  frontMemMm_ = -1;
  return -1;
}

raton::Walls RobotHW::readWalls() {
  lasers_.waitFresh(2);
  auto wall = [](int mm, int threshold) { return mm >= 0 && mm < threshold; };
  return raton::Walls{wall(lasers_.mm(Lasers::LEFT), sideWall_),
                      wall(frontMm(), cfg::FRONT_WALL_MM),
                      wall(lasers_.mm(Lasers::RIGHT), sideWall_)};
}

bool RobotHW::lateralOffset(float& offset, float& rate, float& conf) const {
  const int l = lasers_.mm(Lasers::LEFT);
  const int r = lasers_.mm(Lasers::RIGHT);
  const bool hasL = l >= 0 && l < sideWall_;
  const bool hasR = r >= 0 && r < sideWall_;
  const float center = 0.5f * (corridorMm_ - cfg::ROBOT_WIDTH_MM);
  const float cl = lasers_.confidence(Lasers::LEFT), cr = lasers_.confidence(Lasers::RIGHT);
  if (hasL && hasR) {
    offset = (l - r) * 0.5f;
    rate = (lasers_.rate(Lasers::LEFT) - lasers_.rate(Lasers::RIGHT)) * 0.5f;
    conf = fminf(cl, cr);
  } else if (hasL) {
    offset = l - center;
    rate = lasers_.rate(Lasers::LEFT);
    conf = cl;
  } else if (hasR) {
    offset = center - r;
    rate = -lasers_.rate(Lasers::RIGHT);
    conf = cr;
  } else {
    return false;
  }
  return true;
}

bool RobotHW::lateralOffset(float& offset) const {
  const int l = lasers_.mm(Lasers::LEFT);
  const int r = lasers_.mm(Lasers::RIGHT);
  const bool hasL = l >= 0 && l < sideWall_;
  const bool hasR = r >= 0 && r < sideWall_;
  // Con una sola pared: centrado = la mitad del hueco que deja el raton en el pasillo medido.
  const float center = 0.5f * (corridorMm_ - cfg::ROBOT_WIDTH_MM);
  if (hasL && hasR) {
    offset = (l - r) * 0.5f;
  } else if (hasL) {
    offset = l - center;
  } else if (hasR) {
    offset = center - r;
  } else {
    return false;
  }
  return true;
}

void RobotHW::moveForward(int cells) {
  runCells(cells, mode_ == raton::RunMode::Speed ? cfg::SPEED_RUN_SPEED : speed_, nullptr);
}

int RobotHW::runCells(int cells, float vmax, bool* unexpected) {
  if (unexpected) *unexpected = false;
  if (cells <= 0) return 0;
  if(!actionAllowed(0)) {
    aborted_=true;
    recordMotion("cells",cells*cfg::CELL_MM,{false,0,0,raton::MotionReason::ObservedObstacle});
    if(unexpected)*unexpected=true;
    return 0;
  }
  // Si el giro anterior ya adelanto parte de la celda, se descuenta (ver AXLE_BEHIND_CENTER_MM).
  const float already = pendingAdvance_;
  pendingAdvance_ = 0;
  const float target = fmaxf(cells * cfg::CELL_MM - already, 0.0f);
  const float dt = cfg::CONTROL_PERIOD_US * 1e-6f;

  setState("CELDA");
  encoders_.reset();
  resetHeadingToGrid();
  yawTarget_ = 0;
  headingRef_ = 0;

  float v = 0;          // velocidad de consigna
  float lastPos = 0;
  float speed = 0;      // velocidad medida filtrada
  float integral = 0;
  float pos = 0;
  uint32_t movingAt = millis();  // si se atasca (empujando una pared), deja de empujar
  uint32_t next = micros();

  while (!aborted_) {
    waitPeriod(next);

    pos = 0.5f * (encoders_.left() + encoders_.right()) * cfg::MM_PER_COUNT;
    speed = cfg::SPEED_FILTER * speed + (1.0f - cfg::SPEED_FILTER) * (pos - lastPos) / dt;
    lastPos = pos;
    lastSpeed_ = speed;
    updateHeading();
    if (speed > 20.0f) movingAt = millis();
    if (millis() - movingAt > cfg::STALL_MS) {
      setState("ATASCO");
      if (unexpected) *unexpected = true;
      break;
    }

    float remaining = target - pos;
    const int front = frontMm();
    const int stop = deadzone_ + cfg::FRONT_MARGIN_MM;
    if (front >= 0 && front < cfg::FRONT_WALL_MM) {
      if (remaining > front + cfg::CELL_MM / 2) {
        // Pared delante mucho antes de lo esperado: el mapa no coincide. Parar ante ella.
        if (unexpected) *unexpected = true;
        remaining = static_cast<float>(front - stop);
      } else if (remaining < cfg::CELL_MM) {
        // Ultima celda con pared delante: frenar por el laser corrige el error acumulado.
        remaining = static_cast<float>(front - stop);
      }
    }
    if (remaining <= 1.0f) break;

    // Perfil trapezoidal: acelera hasta vmax y frena para llegar justo.
    const float brakeSpeed = sqrtf(2.0f * cfg::ACCEL * remaining);
    v = fminf(fminf(v + cfg::ACCEL * dt, vmax), fmaxf(brakeSpeed, cfg::MIN_SPEED));

    ctrlActive_ = true;
    controlNetStep(remaining, false);
    controlStep(fmaxf(v * ctrlScale_, cfg::MIN_SPEED), speed, integral, dt);
  }
  ctrlActive_ = false;
  stopMotors();
  const auto progress = raton::motionProgress(pos, already, cfg::CELL_MM, cells, false);
  const int moved = progress.cells;
  // Donde queda el centro del raton respecto al de la celda: con pared delante lo dice el laser; si no,
  // lo que se paso o le falto por encoders. Lo usan el giro y la marcha atras siguientes.
  lasers_.waitFresh(1);
  const int f = lasers_.mm(Lasers::FRONT);
  if (f >= 0 && f < cfg::FRONT_WALL_MM) {
    pendingAdvance_ = 0.5f * corridorMm_ - f - cfg::FRONT_SENSOR_AHEAD_MM + cfg::AXLE_BEHIND_CENTER_MM;
  } else {
    pendingAdvance_ = progress.offset;
  }
  const bool complete=!aborted_ && moved>=cells && !(unexpected && *unexpected);
  recordMotion("cells",cells*cfg::CELL_MM,{complete,pos,gridError(),aborted_?raton::MotionReason::Aborted:
               (complete?raton::MotionReason::Complete:raton::MotionReason::Interrupted)});
  return moved < cells ? moved : cells;
}

int RobotHW::reverseCells(int cells, float shortMm, bool* stuck) {
  if (stuck) *stuck = false;
  if (cells <= 0) return 0;
  const float ofs = pendingAdvance_;
  const float target = fmaxf(cells * cfg::CELL_MM + ofs - shortMm, 0.0f);
  setState("ATRAS");
  encoders_.reset();
  resetHeadingToGrid();
  uint32_t next = micros();
  uint32_t movingAt = millis();
  float back = 0, last = 0;
  const float dt = cfg::CONTROL_PERIOD_US * 1e-6f;
  while (!aborted_) {
    waitPeriod(next);
    back = -0.5f * (encoders_.left() + encoders_.right()) * cfg::MM_PER_COUNT;
    if (back >= target - 1.0f) break;
    if ((back - last) / dt > 20.0f) movingAt = millis();
    last = back;
    if (millis() - movingAt > cfg::STALL_MS) {
      if (stuck) *stuck = true;  // algo detras que no estaba en el mapa
      break;
    }
    updateHeading();
    // Mantener el rumbo de la cuadricula; sin centrar con los laterales (marcha atras se corrige al reves).
    // La red de control, si esta encendida, ajusta ese rumbo (con el signo de marcha atras) y la potencia.
    wallTarget_ = 0;
    controlNetStep(target - back, true);
    const float steer = cfg::KP_YAW * (-ctrlYaw_ - heading_) - cfg::KD_YAW * gyro_.rate();
    const float pwm = (target - back) < 30.0f ? cfg::PWM_MIN_MOVE
                                              : fmaxf(cfg::PWM_MIN_MOVE, cfg::BACKUP_PWM * ctrlScale_);
    motors_.set(-pwm - steer, -pwm + steer);
  }
  stopMotors();
  const auto progress = raton::motionProgress(back, ofs, cfg::CELL_MM, cells, true);
  pendingAdvance_ = progress.offset;
  const int moved = progress.cells;
  const bool complete=!aborted_ && moved>=cells && !(stuck && *stuck);
  recordMotion("reverse",-cells*cfg::CELL_MM,{complete,-back,gridError(),aborted_?raton::MotionReason::Aborted:
               (complete?raton::MotionReason::Complete:raton::MotionReason::Interrupted)});
  return moved < cells ? moved : cells;
}

raton::Walls RobotHW::senseCell() {
  int best[3] = {-1, -1, -1};
  for (int k = 0; k < 5; ++k) {
    lasers_.waitFresh(1);
    for (int i = 0; i < 3; ++i) {
      const int v = lasers_.mm(static_cast<Lasers::Id>(i));
      if (v >= 0 && (best[i] < 0 || v < best[i])) best[i] = v;
    }
  }
  auto wall = [&](int i, int threshold) {
    if (best[i] >= 0) return best[i] < threshold;
    // Ninguna lectura: si la ultima valida era muy corta y fue hace menos de 4 cm de recorrido, esta
    // pegado a la pared (el VL53L0X no mide tan cerca). En un hueco de verdad fue hace casi una celda.
    const Lasers::Id id = static_cast<Lasers::Id>(i);
    return lasers_.lastValidMm(id) < cfg::SIDE_CLOSE_MM && fabsf(odoDist_ - lastValidDist_[i]) < 40.0f;
  };
  return raton::Walls{wall(Lasers::LEFT, cfg::CELL_WALL_MM), wall(Lasers::FRONT, cfg::CELL_WALL_MM),
                      wall(Lasers::RIGHT, cfg::CELL_WALL_MM)};
}

void RobotHW::controlStep(float v, float speed, float& integral, float dt, bool useWalls) {
  const float err = v - speed;
  integral = clampf(integral + err * dt, -50.0f, 50.0f);
  // Los N20 no se mueven por debajo de cierta potencia: en marcha siempre llevan PWM_MIN_RUN y sobre
  // eso se suma lo que pide la velocidad. Sin esto avanzaba a tirones (arranca, se para, arranca...).
  float base = 0;
  if (v > 0) {
    base = cfg::PWM_MIN_RUN + cfg::KFF_SPEED * v + cfg::KP_SPEED * err + cfg::KI_SPEED * integral;
    if (speed < 20.0f && base < cfg::PWM_MIN_MOVE) base = cfg::PWM_MIN_MOVE;  // parado: empujon para arrancar
    if (base < cfg::PWM_MIN_RUN) base = cfg::PWM_MIN_RUN;
  }

  // Direccion en capas:
  //  - con paredes: PID de desviacion lateral (KP_STEER, KD_STEER) con ganancias escaladas por la
  //    confianza de los laseres (si un laser da fantasmas, se le hace menos caso);
  //  - sin paredes (o laseres poco fiables): mantener el rumbo con el giroscopio (KP_YAW).
  // En ambos casos el giroscopio amortigua (KD_YAW). Positivo = girar a la izquierda.
  float steer;
  float offset, rate, conf;
  const bool walls = useWalls && lateralOffset(offset, rate, conf) && conf > 0.2f;
  if (walls) {
    if (fabsf(offset) <= cfg::LATERAL_DEADBAND_MM) {
      offset = 0;
    } else {
      offset -= offset > 0 ? cfg::LATERAL_DEADBAND_MM : -cfg::LATERAL_DEADBAND_MM;
    }
  }
  if (steerMode_ == 1) {
    // PID directo sobre la desviacion (KP_STEER, KD_STEER), escalado por la confianza.
    if (walls) {
      steer = conf * (kpSteer_ * offset + kdSteer_ * rate * dt) - cfg::KD_YAW * gyro_.rate();
      headingRef_ = heading_;
    } else {
      steer = cfg::KP_YAW * (headingRef_ - heading_) - cfg::KD_YAW * gyro_.rate();
    }
  } else {
    // Cascada: la desviacion fija el rumbo objetivo (con menos fuerza si los laseres son poco fiables)
    // y el PID del giroscopio lo sigue.
    float target = 0;
    if (walls) {
      target = clampf(conf * kpWall_ * offset, -cfg::MAX_WALL_CORRECTION_DEG, cfg::MAX_WALL_CORRECTION_DEG);
    }
    wallTarget_ = target;
    if (ctrlActive_) {
      const float lim = 1.5f * cfg::MAX_WALL_CORRECTION_DEG;
      target = clampf(target + ctrlYaw_, -lim, lim);
    }
    yawTarget_ += cfg::YAW_TARGET_SMOOTH * (target - yawTarget_);
    steer = cfg::KP_YAW * (yawTarget_ - heading_) - cfg::KD_YAW * gyro_.rate();
  }
  steer = clampf(steer, -cfg::MAX_STEER, cfg::MAX_STEER);

  motors_.set(base - steer, base + steer);
}

RobotHW::DriveEvent RobotHW::drive(float maxMm, int watchSide) {
  const float dt = cfg::CONTROL_PERIOD_US * 1e-6f;

  setState(watchSide != 0 ? "RECTA" : "ESQUINA");
  encoders_.reset();
  resetHeading();

  float v = 0;
  float lastPos = 0;
  float speed = 0;
  float integral = 0;
  float openSince = -1;  // posicion donde se vio abierto el lado vigilado
  float openSinceL = -1;  // con kWatchBoth, uno por lado
  float openSinceR = -1;
  int lastOkL = 100;  // ultima lectura valida de cada lateral
  int lastOkR = 100;
  float ignoreSide = 0;
  if (watchSide != 0) {
    ignoreSide = ignoreSideMm_;
    ignoreSideMm_ = 0;
  }
  yawTarget_ = 0;
  headingRef_ = 0;
  uint32_t movingAt = millis();  // ultima vez que se movia: si se atasca, se trata como pared
  const uint32_t startMs = millis();
  uint32_t fastAt = 0;  // ultima vez que iba en marcha de verdad (para detectar un frenazo por golpe)
  DriveEvent ev = DriveEvent::Distance;
  uint32_t next = micros();

  while (true) {
    if (aborted_) {
      ev = DriveEvent::Aborted;
      break;
    }
    waitPeriod(next);

    const float pos = 0.5f * (encoders_.left() + encoders_.right()) * cfg::MM_PER_COUNT;
    speed = cfg::SPEED_FILTER * speed + (1.0f - cfg::SPEED_FILTER) * (pos - lastPos) / dt;
    lastPos = pos;
    lastSpeed_ = speed;
    updateHeading();

    if (speed > 20.0f) movingAt = millis();
    if (millis() - movingAt > cfg::STALL_MS) {
      ev = DriveEvent::FrontWall;
      break;
    }
    // Golpe: pico del acelerometro estando en marcha y frenazo de golpe (el tiron al arrancar no cuenta).
    if (speed > cfg::BUMP_MIN_SPEED) fastAt = millis();
    const bool suddenStop = millis() - fastAt < 150 && speed < 0.5f * cfg::BUMP_MIN_SPEED;
    if (millis() - startMs > cfg::BUMP_IGNORE_MS && suddenStop && gyro_.bumpG() > cfg::BUMP_G) {
      ev = DriveEvent::FrontWall;
      setState("CHOQUE");
      break;
    }

    if (goalSet_ && hypotf(odoX_ - goalX_mm_, odoY_ - goalY_mm_) < cfg::GOAL_RADIUS_MM) {
      goalReached_ = true;
      break;
    }
    float remaining = maxMm - pos;
    if (remaining <= 1.0f) break;
    const int front = frontMm();
    const int left = lasers_.mm(Lasers::LEFT);
    const int right = lasers_.mm(Lasers::RIGHT);
    // Rozando una pared lateral: se para para enderezarse hacia el otro lado. Si los dos laterales
    // estan cerca a la vez es un pasillo estrecho, no un roce: solo se centra (si no, se enderezaria
    // a un lado y al otro sin fin).
    const bool leftClose = left >= 0 && left < cfg::SIDE_CRASH_MM;
    const bool rightClose = right >= 0 && right < cfg::SIDE_CRASH_MM;
    const bool narrow = left >= 0 && right >= 0 && abs(left - right) < cfg::SIDE_CRASH_MARGIN_MM;
    // En el tramo diagonal de un giro los laterales ven esquinas y postes: ni roces ni centrado.
    if (!cornerLeg_ && leftClose && !narrow && (right < 0 || left < right)) {
      ev = DriveEvent::CrashLeft;
      setState("ROCE");
      break;
    }
    if (!cornerLeg_ && rightClose && !narrow && (left < 0 || right < left)) {
      ev = DriveEvent::CrashRight;
      setState("ROCE");
      break;
    }
    // Con pared delante cerca, los laterales pueden estar viendo esa misma pared o la esquina:
    // no se usan para centrar ni para buscar huecos.
    const bool frontNear = front >= 0 && front < cfg::FRONT_WALL_MM;
    if (front >= 0) {
      const float toWall = static_cast<float>(front - frontTurnMm());
      if (toWall <= 0) {
        ev = DriveEvent::FrontWall;
        break;
      }
      remaining = fminf(remaining, toWall);
    }

    // Con pared a los dos lados se va midiendo el ancho del pasillo (adapta el raton a la pista).
    if (!frontNear && left >= 0 && right >= 0 && left <= sideWall_ && right <= sideWall_) {
      const float width = left + right + cfg::ROBOT_WIDTH_MM;
      corridorMm_ = clampf(0.98f * corridorMm_ + 0.02f * width, 80.0f, 400.0f);
    }

    // Hueco en el lado vigilado: se confirma tras OPEN_CONFIRM_MM para ignorar lecturas sueltas.
    // Con kWatchBoth: hueco a cualquiera de los dos lados (borde de la pared que termina).
    const bool sidesReady = pos >= ignoreSide;
    if (left >= 0) lastOkL = left;
    if (right >= 0) lastOkR = right;
    // Sin lectura justo despues de una lectura muy corta: demasiado cerca para el laser, no un hueco.
    const int seenL = (left < 0 && lastOkL < cfg::SIDE_CLOSE_MM) ? lastOkL : left;
    const int seenR = (right < 0 && lastOkR < cfg::SIDE_CLOSE_MM) ? lastOkR : right;
    if (watchSide == kWatchBoth && !frontNear && sidesReady) {
      auto check = [&](int mm, float& since) {
        if (mm >= 0 && mm <= sideWall_) {
          since = -1;
          return false;
        }
        if (since < 0) since = pos;
        return pos - since >= cfg::OPEN_CONFIRM_MM;
      };
      const bool l = check(seenL, openSinceL);
      const bool r = check(seenR, openSinceR);
      if (l || r) {
        openSide_ = l ? 1 : -1;
        ev = DriveEvent::SideOpen;
        break;
      }
    } else if (watchSide != 0 && watchSide != kWatchBoth && !frontNear && sidesReady) {
      const int side = watchSide > 0 ? seenL : seenR;
      if (side >= 0 && side <= sideWall_) {
        openSince = -1;
      } else if (openSince < 0) {
        openSince = pos;
      } else if (pos - openSince >= cfg::OPEN_CONFIRM_MM) {
        ev = DriveEvent::SideOpen;
        break;
      }
    }

    const float brakeSpeed = sqrtf(2.0f * cfg::ACCEL * remaining);
    // La velocidad baja si los laseres son poco fiables (la red ajusta la consigna del PID de velocidad).
    const float confAll = fminf(lasers_.confidence(Lasers::FRONT),
                                fmaxf(lasers_.confidence(Lasers::LEFT), lasers_.confidence(Lasers::RIGHT)));
    const float confScale = cfg::CONF_MIN_SPEED + (1.0f - cfg::CONF_MIN_SPEED) * confAll;
    const float vmax = (frontNear ? fminf(speed_, cfg::SLOW_NEAR_WALL_SPEED) : speed_) * confScale;
    v = fminf(fminf(v + cfg::ACCEL * dt, vmax), fmaxf(brakeSpeed, cfg::MIN_SPEED));
    controlStep(v, speed, integral, dt, !frontNear && !cornerLeg_);
  }
  stopMotors();
  const float actual=0.5f*(encoders_.left()+encoders_.right())*cfg::MM_PER_COUNT;
  recordMotion(cornerLeg_?"diagonal":"drive",maxMm,{ev==DriveEvent::Distance,actual,heading_,
    ev==DriveEvent::Aborted?raton::MotionReason::Aborted:(ev==DriveEvent::Distance?
    raton::MotionReason::Complete:raton::MotionReason::Interrupted)});
  return ev;
}

void RobotHW::selfCheck(char* msg) {
  setState("REVISION");
  // 1) Motores: la orden "girar a la izquierda" tiene que dar yaw positivo.
  gyro_.update();
  gyro_.resetYaw();
  encoders_.reset();
  uint32_t t0 = millis();
  motors_.set(-cfg::TURN_PWM_MAX, cfg::TURN_PWM_MAX);
  while (millis() - t0 < 250) gyro_.update();
  motors_.brake();
  t0 = millis();
  while (millis() - t0 < 200) gyro_.update();
  motors_.coast();
  const float yaw = gyro_.yaw();
  const bool motorsSwapped = yaw < -3.0f;
  if (motorsSwapped) motors_.setSwapped(true);
  // Distancia entre ruedas: lo que avanza una rueda menos la otra, entre el angulo girado.
  if (!motorsSwapped && yaw > 10.0f) {
    const float diff = (encoders_.right() - encoders_.left()) * cfg::MM_PER_COUNT;
    const float tw = diff / (yaw / 57.2958f);
    if (tw > 40.0f && tw < 160.0f) trackWidth_ = tw;
  }
  rotate(-yaw);  // volver al rumbo inicial (ya con los motores bien)

  // 2) Laseres: tras girar 90 grados a la izquierda, lo que estaba delante queda a la derecha.
  lasers_.waitFresh(3);
  const int f0 = lasers_.mm(Lasers::FRONT);
  bool lasersSwapped = false;
  if (f0 > 0 && f0 < 1000) {
    rotate(90.0f);
    lasers_.waitFresh(3);
    const int l = lasers_.mm(Lasers::LEFT);
    const int r = lasers_.mm(Lasers::RIGHT);
    rotate(-90.0f);
    const int dl = l < 0 ? 10000 : abs(l - f0);
    const int dr = r < 0 ? 10000 : abs(r - f0);
    if (dl + 40 < dr) {
      lasersSwapped = true;
      lasers_.setSwapped(true);
    }
  }
  snprintf(msg, 22, "Mot %s Las %s", motorsSwapped ? "CRUZ" : "ok", lasersSwapped ? "CRUZ" : "ok");
}

void RobotHW::turnCorner(float degrees, bool trackCells) {
  if (!cfg::TURN_TWO_STAGE || fabsf(fabsf(degrees) - 90.0f) > 1.0f) {
    rotate(degrees);
    return;
  }
  if(!actionAllowed(degrees)) {
    aborted_=true;
    recordMotion("turn",degrees,{false,0,0,raton::MotionReason::ObservedObstacle});
    return;
  }
  lasers_.waitFresh(1);
  raton::SensorObservation observations[3];
  for(int i=0;i<3;++i)observations[i]=sensorObservation(i);
  raton::BodyGeometry body;
  body.sideAhead=cfg::SIDE_SENSOR_AHEAD_MM;body.frontAhead=cfg::FRONT_SENSOR_AHEAD_MM;
  body.sensorWidth=cfg::ROBOT_WIDTH_MM;
  const auto assessment=raton::assessTurn(observations,body,degrees,cfg::TURN_SPLIT_DEG,cfg::TURN_SPLIT_ADVANCE_MM);
  char guardLine[112];
  snprintf(guardLine,sizeof(guardLine),"SWEEP observed=%d feasible=%d entry=%d enabled=%d",assessment.observed,
           assessment.feasible,static_cast<int>(assessment.entryMm),raton::kMotionGuardEnabled);
  telemetry::println(guardLine);
  if(raton::kMotionGuardEnabled && !assessment.feasible) {
    aborted_=true;
    setState("BARRIDO_FALLO");
    recordMotion("turn",degrees,{false,0,0,raton::MotionReason::ObservedObstacle});
    return;
  }
  if(raton::kMotionGuardEnabled && assessment.entryMm<0) backUp(-assessment.entryMm);
  else if(raton::kMotionGuardEnabled && assessment.entryMm>0) drive(assessment.entryMm,0);
  if(raton::kMotionGuardEnabled && assessment.entryMm!=0) {
    const float actual=0.5f*(encoders_.left()+encoders_.right())*cfg::MM_PER_COUNT;
    if(fabsf(actual-assessment.entryMm)>3) {
      aborted_=true;
      recordMotion("entry",assessment.entryMm,{false,actual,0,raton::MotionReason::Interrupted});
    }
  }
  if(aborted_)return;
  const float initialYaw=gyro_.totalYaw();
  const float sign = degrees > 0 ? 1.0f : -1.0f;
  rotate(sign * cfg::TURN_SPLIT_DEG);
  if (aborted_) return;
  cornerLeg_ = true;
  drive(cfg::TURN_SPLIT_ADVANCE_MM, 0);
  cornerLeg_ = false;
  const float cornerDistance = 0.5f * (encoders_.left() + encoders_.right()) * cfg::MM_PER_COUNT;
  if (aborted_) return;
  if (gridOn_) {
    // Segundo tiempo hasta el rumbo exacto de la cuadricula (no un angulo fijo): no acumula errores.
    gridYaw_ += sign * 90.0f;
    float e = fmodf(gridYaw_ - gyro_.totalYaw(), 360.0f);
    if (e > 180.0f) e -= 360.0f;
    if (e < -180.0f) e += 360.0f;
    rotate(e);
  } else {
    rotate(sign * (90.0f - cfg::TURN_SPLIT_DEG));
  }
  if (aborted_) return;
  if (trackCells) {
    // El eje avanzo la distancia medida por sin(split) en la direccion nueva; la referencia de
    // celda centrada lo tiene AXLE_BEHIND_CENTER_MM por detras del centro.
    pendingAdvance_ = cornerDistance * sinf(cfg::TURN_SPLIT_DEG / 57.2958f) + cfg::AXLE_BEHIND_CENTER_MM;
  }
  recordMotion("turn",degrees,{true,cornerDistance,degrees-(gyro_.totalYaw()-initialYaw),raton::MotionReason::Complete});
}

int RobotHW::backOut(float maxMm) {
  setState("SALIDA");
  encoders_.reset();
  resetHeading();
  const uint32_t t0 = millis();
  uint32_t next = micros();
  float sinceL = -1;
  float sinceR = -1;
  int found = 0;
  float foundAt = 0;
  while (!aborted_ && millis() - t0 < 5000) {
    waitPeriod(next);
    const float back = -0.5f * (encoders_.left() + encoders_.right()) * cfg::MM_PER_COUNT;
    if (back >= maxMm) break;
    if (found == 0) {
      auto check = [&](int mm, float& since) {
        if (mm >= 0 && mm <= sideWall_) {
          since = -1;
          return false;
        }
        if (since < 0) since = back;
        return back - since >= cfg::OPEN_CONFIRM_MM;
      };
      if (check(lasers_.mm(Lasers::LEFT), sinceL)) found = 1;
      else if (check(lasers_.mm(Lasers::RIGHT), sinceR)) found = -1;
      if (found) foundAt = back;
    } else if (back - foundAt >= 0.5f * corridorMm_) {
      break;  // el laser lateral ya esta en el centro del hueco
    }
    updateHeading();
    const float steer = cfg::KP_YAW * -heading_ - cfg::KD_YAW * gyro_.rate();
    motors_.set(-cfg::BACKUP_PWM - steer, -cfg::BACKUP_PWM + steer);
  }
  stopMotors();
  const float actual=0.5f*(encoders_.left()+encoders_.right())*cfg::MM_PER_COUNT;
  recordMotion("back_out",-maxMm,{!aborted_ && found!=0,actual,heading_,aborted_?
    raton::MotionReason::Aborted:(found?raton::MotionReason::Complete:raton::MotionReason::Interrupted)});
  return found;
}

void RobotHW::backUp(float mm) {
  setState("ATRAS");
  encoders_.reset();
  resetHeading();
  const uint32_t t0 = millis();
  uint32_t next = micros();
  while (!aborted_ && millis() - t0 < 1000) {
    waitPeriod(next);
    const float pos = 0.5f * (encoders_.left() + encoders_.right()) * cfg::MM_PER_COUNT;
    if (pos <= -mm) break;
    updateHeading();
    const float steer = cfg::KP_YAW * -heading_ - cfg::KD_YAW * gyro_.rate();
    motors_.set(-cfg::BACKUP_PWM - steer, -cfg::BACKUP_PWM + steer);
  }
  stopMotors();
  const float actual=0.5f*(encoders_.left()+encoders_.right())*cfg::MM_PER_COUNT;
  const bool complete=!aborted_ && actual<=-mm+1;
  recordMotion("back",-mm,{complete,actual,heading_,complete?raton::MotionReason::Complete:
    (aborted_?raton::MotionReason::Aborted:raton::MotionReason::Interrupted)});
}

void RobotHW::rotate(float degrees) {
  setState("GIRO");
  frontMemMm_ = -1;  // al girar, la pared recordada ya no esta delante
  gyro_.update();
  gyro_.resetYaw();
  const uint32_t t0 = millis();
  uint32_t next = micros();
  int settled = 0;

  while (!aborted_ && millis() - t0 < cfg::TURN_TIMEOUT_MS) {
    waitPeriod(next);
    const float err = degrees - gyro_.yaw();
    const float rate = gyro_.rate();
    // Dentro de tolerancia: frenar y dar el giro por bueno en cuanto se quede casi quieto.
    // Oscilar no demuestra que el giro se haya completado.
    if (fabsf(err) < cfg::TURN_TOLERANCE_DEG) {
      motors_.brake();
      if (fabsf(rate) < 30.0f && ++settled > 3) break;
      continue;
    }
    settled = 0;
    const int sign = err >= 0 ? 1 : -1;
    float pwm = clampf(cfg::KP_TURN * err - cfg::KD_TURN * rate, -cfg::TURN_PWM_MAX, cfg::TURN_PWM_MAX);
    // El minimo solo hace falta para arrancar: girando ya, se deja bajar y frena sin pasarse.
    if (fabsf(rate) < 30.0f && fabsf(pwm) < cfg::TURN_PWM_MIN) pwm = sign * cfg::TURN_PWM_MIN;
    if (cfg::TURN_IN_PLACE) {
      motors_.set(-pwm, pwm);  // sobre si mismo: lo que menos espacio ocupa
    } else if (pwm > 0) {
      // Pivote sobre una rueda: a la izquierda se para la izquierda y empuja la derecha.
      motors_.set(0, pwm);
    } else {
      motors_.set(-pwm, 0);
    }
  }
  stopMotors();
  if (!aborted_ && fabsf(degrees - gyro_.yaw()) >= cfg::TURN_TOLERANCE_DEG) {
    aborted_ = true;
    setState("GIRO_FALLO");
    telemetry::println("ERROR: giro incompleto; parado para no perder la posicion");
  }
  recordMotion("rotate",degrees,{!aborted_,0,degrees-gyro_.yaw(),aborted_?
    raton::MotionReason::TurnError:raton::MotionReason::Complete});
}

void RobotHW::onStep(const raton::Maze& maze, const raton::FloodFill& flood, raton::Cell pos, raton::Dir heading) {
  // Puerta del mapa: comparar las paredes que acaba de ver (izq, delante, der) con el mapa conocido.
  if (expected_ && mode_ == raton::RunMode::Search) {
    const raton::Dir dirs[3] = {raton::leftOf(heading), heading, raton::rightOf(heading)};
    for (raton::Dir d : dirs) {
      if (!maze.isKnown(pos, d)) continue;
      mapSeen_++;
      if (maze.hasWall(pos, d) == expected_(pos.x, pos.y, d)) mapHits_++;
    }
    if (!mapWarned_ && mapSeen_ >= 3 * cfg::MAP_CONF_STEPS && mapConfidence() < cfg::MAP_CONF_MIN) {
      mapWarned_ = true;
      speed_ *= 0.7f;
      char line[80];
      snprintf(line, sizeof(line), "DECIDE: el mapa coincide solo un %d%% -> modo prudente (mas despacio)",
               static_cast<int>(mapConfidence() * 100));
      telemetry::println(line);
    }
  }
  char l1[22];
  char l2[22];
  char l3[22];
  snprintf(l1, sizeof(l1), "Celda %d,%d  %c", pos.x, pos.y, raton::dirChar(heading));
  snprintf(l2, sizeof(l2), "Dist meta: %u", static_cast<unsigned>(flood.distance(pos)));
  snprintf(l3, sizeof(l3), "I%4d F%4d D%4d", lasers_.mm(Lasers::LEFT), lasers_.mm(Lasers::FRONT),
           lasers_.mm(Lasers::RIGHT));
  display_.show(mode_ == raton::RunMode::Speed ? "CARRERA" : "EXPLORANDO", l1, l2, l3);
  Serial.print("celda ");
  Serial.print(pos.x);
  Serial.print(',');
  Serial.print(pos.y);
  Serial.print(" rumbo ");
  Serial.print(raton::dirChar(heading));
  Serial.print(" dist ");
  Serial.println(flood.distance(pos));
}
