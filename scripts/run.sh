#!/usr/bin/env bash
# Corre el entorno del proyecto. Requiere haber ejecutado antes scripts/setup.sh.
#
#   scripts/run.sh                  pruebas rapidas y abre la web de telemetria (http://127.0.0.1:5173)
#   scripts/run.sh web              solo la web en modo desarrollo
#   scripts/run.sh pruebas          todas las pruebas: nucleo C++, simulador, web y compilacion del firmware
#                                   (con --sin-firmware no compila el firmware)
#   scripts/run.sh firmware         compila el firmware (raton y prueba_hardware)
#   scripts/run.sh subir [puerto]   compila y sube el firmware a la placa por USB
#   scripts/run.sh monitor [puerto] Monitor Serie de la placa
#   scripts/run.sh sim <orden> ...  simulador: casos, pistas, evaluar, publicar, pesos, entrenar
#   scripts/run.sh ayuda            esta ayuda
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

paso() { printf '\n\033[1m==> %s\033[0m\n' "$*"; }

PY="$ROOT/.venv/bin/python"
PIO="$ROOT/.venv/bin/pio"
if [ ! -x "$PY" ] || [ ! -d web/node_modules ]; then
  echo "El entorno no esta instalado: ejecuta primero scripts/setup.sh" >&2
  exit 1
fi
[ -x "$PIO" ] || PIO="$(command -v pio || echo "$HOME/.platformio/penv/bin/pio")"

native_lib() {
  for f in build/libraton_navigation.dylib build/libraton_navigation.so build/raton_navigation.dll; do
    [ -f "$f" ] && { echo "$ROOT/$f"; return; }
  done
  echo ""
}
export RATON_NATIVE_LIB="${RATON_NATIVE_LIB:-$(native_lib)}"

nucleo() {
  paso "Nucleo C++"
  cmake -S . -B build
  cmake --build build --parallel
  ctest --test-dir build --output-on-failure
}

simulador() {
  paso "Simulador Python"
  "$PY" -m ruff check simulator/python tests/python
  "$PY" -m ruff format --check simulator/python tests/python
  "$PY" -m pytest tests/python -q
}

web_checks() {
  paso "Web"
  (cd web && npm run lint && npm run format:check && npm run typecheck && npm test && npm run build)
}

firmware() {
  paso "Firmware"
  "$PIO" run -e raton -e prueba_hardware
}

web_dev() {
  paso "Web de telemetria en http://127.0.0.1:5173 (Ctrl+C para parar)"
  echo "La conexion por Bluetooth necesita Chrome o Edge (Web Bluetooth)."
  cd web && exec npm run dev
}

case "${1:-}" in
  "")
    nucleo
    simulador
    web_dev
    ;;
  web) web_dev ;;
  pruebas)
    nucleo
    simulador
    web_checks
    [ "${2:-}" = "--sin-firmware" ] || firmware
    paso "Todo correcto"
    ;;
  firmware) firmware ;;
  subir)
    if [ -n "${2:-}" ]; then "$PIO" run -e raton -t upload --upload-port "$2"; else "$PIO" run -e raton -t upload; fi
    ;;
  monitor)
    if [ -n "${2:-}" ]; then "$PIO" device monitor -b 115200 -p "$2"; else "$PIO" device monitor -b 115200; fi
    ;;
  sim)
    shift
    exec "$PY" -m raton_sim "$@"
    ;;
  ayuda|-h|--help) sed -n '2,13p' "$0" | sed 's/^# \{0,1\}//' ;;
  *)
    echo "Orden desconocida: $1 (usa scripts/run.sh ayuda)" >&2
    exit 1
    ;;
esac
