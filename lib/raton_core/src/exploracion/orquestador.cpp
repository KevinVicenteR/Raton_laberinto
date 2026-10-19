#include "exploracion/orquestador.h"

namespace raton {

AccionMision OrquestadorMision::decidir(const EstadoMision& e) const {
  if (e.abortado) return AccionMision::Terminar;
  if (!e.salidaLeida) return AccionMision::LeerSalida;
  if (e.hayMeta) {
    if (e.enMeta) return AccionMision::Terminar;
    if (e.fallosSeguidos >= p_.maxFallos) return AccionMision::Recuperar;
    return e.mapaAprendido ? AccionMision::RutaCorta : AccionMision::Explorar;
  }
  // Sin meta = entrenar: explorar mientras quede frontera.
  if (e.fallosSeguidos >= p_.maxFallos) return AccionMision::Recuperar;
  return e.hayFrontera ? AccionMision::Explorar : AccionMision::Terminar;
}

const char* OrquestadorMision::nombre(AccionMision a) {
  switch (a) {
    case AccionMision::LeerSalida: return "leer salida";
    case AccionMision::Explorar: return "explorar";
    case AccionMision::RutaCorta: return "ruta mas corta";
    case AccionMision::Recuperar: return "recuperar";
    default: return "terminar";
  }
}

}  // namespace raton
