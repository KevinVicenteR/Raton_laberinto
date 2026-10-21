// Parametros del raton. Un solo sitio para todo lo ajustable: el firmware y el simulador usan los mismos
// valores por defecto, asi lo que se prueba en el PC es lo que corre en la placa.
// Unidades: mm, s, grados. "duty" = fraccion de PWM (-1..1).
#pragma once

#include <stdint.h>

namespace raton {

struct Params {
  // ---------- Pista (reglamento SECIHTI 2026, categoria Robot raton) ----------
  // Laberinto ~1.5 x 3 m, pasillos de ~14 cm, paredes de 5 cm de alto.
  float cellMm = 155.0f;  // de centro a centro de celda (pasillo + pared). Se corrige en marcha.
  float wallMm = 12.0f;   // grosor de pared
  int mazeW = 19;         // celdas a lo ancho
  int mazeH = 10;         // celdas a lo alto
  int startX = 18;        // salida (INICIO): abajo a la derecha...
  int startY = 0;
  int startDir = 0;       // ...mirando al norte (0=N 1=E 2=S 3=W)
  int goalX = 4;          // meta del ejemplo del reglamento (configurable con G x,y)
  int goalY = 2;
  int goalW = 1;          // la meta puede ser una zona de goalW x goalH celdas
  int goalH = 1;
  bool knownBounds = true;  // el borde exterior es pared

  // ---------- Mecanica (CALIBRAR) ----------
  float wheelDiamMm = 34.0f;                  // se afina solo en marcha (paredes de frente)
  float countsPerRev = 7.0f * 4.0f * 100.0f;  // N20 1:100, 7 pulsos x 4 flancos
  float trackMm = 75.0f;                      // distancia entre ruedas (se afina sola en los giros)
  float bodyDiamMm = 90.0f;                   // chasis visto como circulo (curvas suaves, simulacion)
  // Chasis visto desde arriba respecto al eje de las ruedas: lo que sobresale por delante y por detras y el
  // ancho. Si las ruedas estan adelante, la cola barre un circulo grande al girar en el sitio.
  float bodyFrontMm = 45.0f;
  float bodyRearMm = 45.0f;
  float bodyWidthMm = 90.0f;

  // ---------- Sensores VL53L0X ----------
  // Frontal en el centro y dos laterales a 45 grados (los opcionales a 90 grados se configuran al final).
  float frontAheadMm = 40.0f;   // posicion del frontal por delante del eje de ruedas
  float diagAheadMm = 30.0f;    // laterales: por delante del eje...
  float diagLateralMm = 30.0f;  // ...y separados del centro hacia cada lado
  float diagMountDeg = 45.0f;   // angulo de montaje de los laterales respecto al avance
  // El VL53L0X ilumina un cono (25 grados): la sonda lo modela entero (ver sonda.h), tambien en vertical:
  // la parte de arriba del cono pasa por encima de las paredes si el sensor esta alto, y la de abajo toca el
  // piso a h / tan(12.5) (con el sensor a 40 mm, a ~18 cm: "ve algo delante" en un pasillo vacio).
  float sensorFovDeg = 25.0f;
  float sensorHeightMm = 40.0f;  // centro de los sensores sobre el piso (MEDIR: set sensorHeightMm)
  float sensorTiltDeg = 0.0f;    // inclinacion de los sensores (+ = hacia arriba)
  float wallHeightMm = 50.0f;    // paredes del laberinto (reglamento: 5 cm)
  float floorReflect = 0.3f;     // cuanto refleja el piso frente a una pared (suelo negro ~0.1, madera ~0.4)
  // Lo que cada sensor mira de mas hacia arriba (izquierdo, frontal, derecho) y el reflejo del piso se
  // aprenden solos en marcha con las paredes ya seguras y se guardan al terminar (learnVertical).
  float leftTiltDeg = 0.0f;
  float frontTiltDeg = 0.0f;
  float rightTiltDeg = 0.0f;
  int frontOffsetMm = 0;        // se suma a cada lectura (calibracion: calsonda / calfront)
  int leftOffsetMm = 0;
  int rightOffsetMm = 0;
  float sensorLatencyS = 0.012f;  // retraso medio de una medida (mitad del presupuesto de 20 ms + I2C)
  int sensorMaxMm = 600;          // mas lejos = no hay nada

  // ---------- Motores (modelo de avance) ----------
  // duty = kS * signo + kV * velocidad + kA * aceleracion (por rueda, mm/s y mm/s^2), con kA = kV * tau.
  // kV: con 8.4 V y PWM 100% un N20 1:100 de 6 V llega a ~750 mm/s -> 1/750. tau = constante de tiempo
  // del motor con el raton en el suelo. Los tres se miden con el modo de calibracion de motores.
  float kS = 0.05f;
  float kV = 1.0f / 750.0f;
  float motorTauS = 0.04f;
  float kA() const { return kV * motorTauS; }
  float maxDuty = 0.95f;

  // ---------- Control (lazo a 500 Hz) ----------
  float dt = 0.002f;
  float kpFwd = 0.012f;    // duty por mm de error de posicion
  float kdFwd = 0.0004f;   // duty por mm/s de error
  float kpRot = 0.020f;    // duty por grado de error de rumbo
  float kdRot = 0.0006f;   // duty por grado/s
  float kiFwd = 0.02f;     // duty por mm*s (integral con limite)
  float kiRot = 0.02f;     // duty por grado*s
  // Centrado con los laterales (cascada): desvio de rumbo por mm de descentrado, limitado.
  float kSteer = 0.25f;          // grados por mm
  float steerMaxDeg = 5.0f;
  float steerRateGain = 0.010f;  // grados por mm/s de deriva lateral

  // ---------- Velocidades ----------
  float searchSpeed = 400.0f;   // explorando (mm/s)
  float knownSpeed = 650.0f;    // explorando, en recta por celdas ya recorridas
  float runSpeed = 650.0f;      // carrera con ruta conocida
  float turnSpeed = 450.0f;     // curvas suaves: tope (manda lo que dan los motores, ver Navigator::arcSpeed)
  float accel = 3000.0f;        // mm/s^2 (0.3 g: las ruedas no patinan)
  float decel = 3000.0f;
  float rotSpeed = 540.0f;      // giro en el sitio (grados/s)
  float rotAccel = 3600.0f;     // grados/s^2
  bool smoothTurnsSearch = false;  // explorando: giro en el centro (robusto) o curva
  bool smoothTurnsRun = true;      // carrera: curvas sin parar
  float smoothRadiusMm = 50.0f;    // radio minimo de las curvas suaves (se agranda solo hasta que el cuerpo cabe)

  // ---------- Paredes ----------
  // Lateral: hay pared si la distancia lateral (desde el centro del raton) es menor que esto x medio pasillo.
  float sideWallRatio = 1.6f;
  // Frontal: hay pared en la celda de delante si el frontal lee menos que la pared esperada + esto x celda.
  float frontWallRatio = 0.45f;
  int frontStopMm = -1;  // lectura del frontal con el raton en el centro de la celda; -1 = de la geometria

  // ---------- Exploracion y seguridad ----------
  float exploreBudgetS = 150.0f;  // tras la meta, como mucho esto buscando la ruta mas corta
  float areaLimitS = 10.0f;       // en la misma zona mas de esto = forzar salida (regla: 30 s)
  int areaRadius = 1;             // radio (celdas) de "misma zona"
  float stallS = 0.35f;           // ruedas sin avanzar con orden de moverse = atasco
  float backoffMm = 35.0f;        // atasco: retroceso antes de enderezarse

  // ---------- Autocalibracion en marcha ----------
  int learnGyro = 1;    // escala del giroscopio con las paredes
  int learnMotor = 1;   // escala del modelo del motor en las rectas (bateria, motores sin calibrar)
  int learnVertical = 1;  // inclinacion de cada sensor y reflejo del piso
  int rangeBiasFromCell = 0;  // experimental: corregir los laseres con el paso de celda aprendido
  float gyroScale = 1.0f;  // escala del giroscopio aprendida (se guarda sola al terminar cada mision)

  // Ajustes aprendidos. Se anaden al final para conservar la configuracion flash anterior.
  float motorBalance = 0.0f;  // trim diferencial: izquierda 1-b, derecha 1+b
  float startBoostLF = 0.0f, startBoostLR = 0.0f;
  float startBoostRF = 0.0f, startBoostRR = 0.0f;
  float learnedCellMm = 0.0f;      // paso en unidades del encoder; no cambia el diametro fisico
  float learnedCellRefMm = 0.0f;   // geometria y escala con las que se aprendio
  float learnedWheelDiamMm = 0.0f;
  float learnedCountsPerRev = 0.0f;
  int precisionMode = 1;  // perfil fisico prudente; 0 habilita las velocidades configuradas

  // Laterales opcionales a 90 grados. Activar SOLO despues de medir el montaje.
  int sideEnabled = 0;
  float sideAheadMm = 0.0f;    // respecto al eje de ruedas (+ delante)
  float sideLateralMm = 30.0f; // distancia del centro a cada lente (montaje simetrico)
  int sideLeftOffsetMm = 0, sideRightOffsetMm = 0;

  // Confirm only after measuring the chassis and sensor geometry. Does not enable
  // lateral fusion by itself. Advanced trajectories remain clearance-gated.
  int autoGeometryConfirmed = 0;


};

}  // namespace raton

