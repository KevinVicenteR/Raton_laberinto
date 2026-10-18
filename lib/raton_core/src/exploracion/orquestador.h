#pragma once
#include <stdint.h>

namespace raton {

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
