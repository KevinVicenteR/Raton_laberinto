// Ajustes que se cambian en marcha por Bluetooth o Serie (ver CommandHandler).
#pragma once

#include "config.h"

struct Tuning {
  float speed = cfg::WALL_FOLLOW_SPEED;  // v<mm/s>: velocidad del seguidor de paredes
  float kpWall = cfg::KP_WALL;           // k: cascada, grados de rumbo por mm descentrado
  float kpSteer = cfg::KP_STEER;         // j: PID directo
  float kdSteer = cfg::KD_STEER;         // d: PID directo
  int steerMode = cfg::STEER_MODE;       // p: 0 cascada, 1 PID directo
  int sideWall = cfg::SIDE_WALL_MM;      // w: costado a menos de esto = pared (seguidor y modo 5)
  // n: pesos de la decision en los cruces del modo 5 [espacio, novedad, giro, sesgo recto].
  float weights[4] = {cfg::W_FREE, cfg::W_NOVEL, cfg::W_TURN, cfg::B_STRAIGHT};
};

// Ordenes recibidas que el bucle principal consume (arrancar, modo, meta).
struct OperatorRequests {
  bool start = false;
  int mode = 0;               // 1..5, 0 = sin cambio
  int goalX = cfg::GOAL_X;    // celda meta pedida desde la web, -1 = ninguna
  int goalY = cfg::GOAL_Y;
};
