// Tabla de parametros por nombre: para ajustarlos por Bluetooth/Serie ("set kpRot 0.02"), guardarlos en
// flash y probarlos en el simulador (--set kpRot=0.02) con el mismo nombre.
#pragma once

#include "raton/params.h"

namespace raton {

struct ParamInfo {
  const char* name;
  bool isInt;
  float min, max;
};

int paramCount();
const ParamInfo& paramInfo(int i);
float getParam(const Params& p, int i);
// Devuelve false si el nombre no existe o el valor esta fuera de rango.
bool setParam(Params& p, const char* name, float value);
int findParam(const char* name);

}  // namespace raton
