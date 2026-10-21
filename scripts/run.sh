#!/usr/bin/env bash
# Ordenes del proyecto.
#
#   scripts/run.sh pruebas            compila y pasa las pruebas (nucleo + misiones simuladas)
#   scripts/run.sh sim [opciones]     simulador (por defecto: laberinto del reglamento con detalle)
#   scripts/run.sh bateria [n]        reglamento + n laberintos aleatorios, ideales y con errores
#   scripts/run.sh firmware           compila el firmware (PlatformIO)
#   scripts/run.sh subir [puerto]     compila y sube el firmware por USB
#   scripts/run.sh monitor [puerto]   Monitor Serie (115200)
#   scripts/run.sh web                web de telemetria por Bluetooth (http://127.0.0.1:5173)
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
paso() { printf '\n\033[1m==> %s\033[0m\n' "$*"; }
BUILD_DIR=build
compilar() {
  local generator="" compiler=""
  case "$(uname -s)" in
    MINGW*|MSYS*|CYGWIN*)
      # CMake de Windows suele elegir NMake aunque no exista en PATH.
      # Usar directorios separados evita reutilizar ese cache incompleto.
      if command -v cl >/dev/null 2>&1; then
        compiler=cl
        if command -v ninja >/dev/null 2>&1; then
          generator=Ninja
          BUILD_DIR=build-windows-msvc-ninja
        elif command -v nmake >/dev/null 2>&1; then
          generator="NMake Makefiles"
          BUILD_DIR=build-windows-msvc-nmake
        fi
      elif command -v g++ >/dev/null 2>&1 && [[ "$(g++ -dumpmachine)" == *mingw* ]]; then
        compiler=g++
        if command -v ninja >/dev/null 2>&1; then
          generator=Ninja
          BUILD_DIR=build-windows-mingw-ninja
        elif command -v mingw32-make >/dev/null 2>&1; then
          generator="MinGW Makefiles"
          BUILD_DIR=build-windows-mingw-make
        fi
      fi
      if [[ -z "$generator" ]]; then
        cat >&2 <<'MSG'
Falta un entorno C++ completo para las pruebas en Windows.
Opcion MSVC: instala Visual Studio Build Tools con "Desarrollo para el escritorio con C++"
y abre "Developer PowerShell for VS". Desde esa terminal ejecuta:
  bash ./scripts/run.sh pruebas
Alternativa: MinGW-w64 (g++) con Ninja o mingw32-make disponibles en PATH.
Git Bash y CMake por si solos no incluyen un compilador C++.
Para compilar la placa con PlatformIO: bash ./scripts/run.sh firmware
MSG
        return 1
      fi
      cmake -S . -B "$BUILD_DIR" -G "$generator" "-DCMAKE_CXX_COMPILER=$compiler" -DCMAKE_BUILD_TYPE=Release
      ;;
    *) cmake -S . -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE=Release ;;
  esac
  cmake --build "$BUILD_DIR" --parallel
}
PIO="$(command -v pio || echo "$HOME/.platformio/penv/bin/pio")"

case "${1:-pruebas}" in
  pruebas)
    paso "Nucleo, firmware (comprobacion en el PC) y misiones simuladas"
    compilar
    ctest --test-dir "$BUILD_DIR" --output-on-failure
    ;;
  sim)
    shift
    compilar >/dev/null
    "$BUILD_DIR/raton_sim" "$@"
    ;;
  bateria)
    compilar >/dev/null
    "$BUILD_DIR/raton_sim" --bench "${2:-20}"
    ;;
  firmware) "$PIO" run ;;
  subir) "$PIO" run -t upload ${2:+--upload-port "$2"} ;;
  monitor) "$PIO" device monitor -b 115200 ${2:+-p "$2"} ;;
  web)
    (cd web && { [ -d node_modules ] || npm ci; } && npm run dev)
    ;;
  *)
    sed -n '2,11p' "$0" | sed 's/^# \{0,1\}//'
    exit 1
    ;;
esac
