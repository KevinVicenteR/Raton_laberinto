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
compilar() {
  cmake -S . -B build -DCMAKE_BUILD_TYPE=Release >/dev/null
  cmake --build build --parallel
}
PIO="$(command -v pio || echo "$HOME/.platformio/penv/bin/pio")"

case "${1:-pruebas}" in
  pruebas)
    paso "Nucleo, firmware (comprobacion en el PC) y misiones simuladas"
    compilar
    ctest --test-dir build --output-on-failure
    ;;
  sim)
    shift
    compilar >/dev/null
    build/raton_sim "$@"
    ;;
  bateria)
    compilar >/dev/null
    build/raton_sim --bench "${2:-20}"
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
