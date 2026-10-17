// Arquitectura de dos orquestadores (sin dependencias de hardware: se prueba en el ordenador).
//
//  OrquestadorMision     decide QUE toca: leer la salida, explorar, ruta mas corta, recuperarse o acabar.
//                        Lleva la politica de fallos (reintentos, celdas inalcanzables).
//  OrquestadorMovimiento decide COMO moverse en cada ciclo de control y vigila la seguridad: frenado
//                        (velocidad segun la pared de delante) y seguridad (roce, torcido, empuje contra
//                        pared, atasco). Puede vetar el movimiento.
#pragma once
#include <stdint.h>

namespace raton {

// ---------------------------------------------------------------- movimiento
struct ParamsMovimiento {
  float sideCrashMm = 8;      // costado a menos de esto = roce
  float skewLimitDeg = 10;    // torcido respecto a la cuadricula
  uint32_t rubMs = 100;       // roce/torcido sostenido antes de actuar
  int maxRubs = 3;            // roces en un tramo antes de darse por pegado
  float pushCheckMm = 40;     // empuje: avanza esto sin que el frontal se acerque...
  float pushMinCloseMm = 10;  // ...al menos esto
  uint32_t stallMs = 600;     // sin moverse con potencia
  float accel = 800;          // mm/s2 para el frenado
  float minSpeed = 40;        // mm/s
};

struct EntradaMovimiento {
  uint32_t ms;           // reloj
  float pos;             // avance en el tramo (mm)
  float speed;           // velocidad medida (mm/s)
  float remaining;       // lo que falta segun encoders (mm)
  int front;             // frontal confirmado (mm, -1 = nada)
  int stopMm;            // lectura frontal a la que parar (raton centrado)
  int wallNearMm;        // por debajo, hay pared delante
  float cellMm;
  int sideL, sideR;      // distancia de cada costado a su pared (mm, -1 = nada)
  float skewDeg;         // error respecto a la cuadricula
};

enum class Seguridad : uint8_t { Ok, Roce, Pegado };

struct DecisionMovimiento {
  float remaining;       // lo que falta tras mirar la pared de delante (mm)
  bool paredInesperada;  // pared delante mucho antes de lo esperado
  float vmaxFrenado;     // velocidad maxima para llegar parado (mm/s)
  Seguridad seguridad;
};

class OrquestadorMovimiento {
 public:
  explicit OrquestadorMovimiento(const ParamsMovimiento& p = ParamsMovimiento()) : p_(p) {}
  void empezarTramo(uint32_t ms);
  // Tras apartarse de un roce: vuelve a vigilar desde cero (cuenta el roce hecho).
  void trasRecuperar(uint32_t ms, float pos);
  DecisionMovimiento decidir(const EntradaMovimiento& in);
  int roces() const { return rubs_; }

 private:
  ParamsMovimiento p_;
  uint32_t rubSince_ = 0, movingAt_ = 0;
  int rubs_ = 0;
  float pushPos_ = -1;
  int pushFront_ = 0;
};

// ---------------------------------------------------------------- mision
enum class AccionMision : uint8_t { LeerSalida, Explorar, RutaCorta, Recuperar, Terminar };

struct EstadoMision {
  bool salidaLeida;      // la celda de salida ya se leyo
  bool hayFrontera;      // queda alguna celda sin ver a la que se puede llegar
  bool hayMeta;          // hay meta elegida
  bool mapaAprendido;    // ya hay un mapa completo guardado
  bool enMeta;           // esta en la meta
  int fallosSeguidos;    // tramos fallidos seguidos hacia el mismo objetivo
  bool abortado;
};

struct ParamsMision {
  int maxFallos = 3;     // fallos con un mismo objetivo antes de darlo por inalcanzable
};

class OrquestadorMision {
 public:
  explicit OrquestadorMision(const ParamsMision& p = ParamsMision()) : p_(p) {}
  AccionMision decidir(const EstadoMision& e) const;
  static const char* nombre(AccionMision a);

 private:
  ParamsMision p_;
};

}  // namespace raton
