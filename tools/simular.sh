#!/usr/bin/env bash
# Corre las simulaciones del raton y abre los resultados en el navegador.
#
# Uso (desde cualquier carpeta):
#   tools/simular.sh            casos extremos (tools/sim_extremos.py) y abre el informe
#   tools/simular.sh pesos [N]  optimiza los pesos de la decision en los cruces con N candidatos (24)
#   tools/simular.sh simulacion abre el laboratorio con resultados y reproducciones
#   tools/simular.sh web        solo sirve la pagina de telemetria y la abre
#
# La pagina y el informe se sirven en http://localhost:8000 (la telemetria Bluetooth necesita Chrome,
# Edge o Brave con Web Bluetooth activado).
set -euo pipefail

TOOLS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PORT=8000
PY="$(command -v python3 || true)"
if [[ -z "$PY" ]]; then
  echo "Hace falta python3." >&2
  exit 1
fi

# Arranca el servidor de tools/ en segundo plano si no esta ya en marcha.
serve() {
  # -f: un 404 cuenta como fallo (otro servidor en el puerto sirviendo otra carpeta).
  if curl -sf -o /dev/null "http://localhost:$PORT/telemetria.html"; then
    return
  fi
  if curl -s -o /dev/null "http://localhost:$PORT/"; then
    echo "El puerto $PORT ya lo usa otro servidor que no sirve tools/ (¿python3 -m http.server lanzado desde otra carpeta?)." >&2
    echo "Paralo con Ctrl+C en su terminal y vuelve a ejecutar este script." >&2
    exit 1
  fi
  (cd "$TOOLS" && nohup "$PY" -m http.server "$PORT" >/dev/null 2>&1 &)
  for _ in $(seq 1 20); do
    curl -sf -o /dev/null "http://localhost:$PORT/telemetria.html" && return
    sleep 0.2
  done
  echo "No se pudo arrancar el servidor en el puerto $PORT." >&2
  exit 1
}

open_url() {
  if command -v open >/dev/null; then open "$1"; elif command -v xdg-open >/dev/null; then xdg-open "$1"; else echo "Abre: $1"; fi
}

case "${1:-casos}" in
  casos)
    echo "Simulando casos extremos (tarda unos minutos)..."
    "$PY" "$TOOLS/sim_extremos.py"
    serve
    open_url "http://localhost:$PORT/sim_resultados/index.html"
    ;;
  pesos)
    echo "Optimizando pesos con ${2:-24} candidatos (tarda unos minutos)..."
    "$PY" "$TOOLS/optimiza_pesos.py" "${2:-24}"
    echo "Copia los pesos a W_FREE, W_NOVEL, W_TURN y B_STRAIGHT en src/config.h o envialos desde la web."
    ;;
  simulacion)
    serve
    open_url "http://localhost:$PORT/simulacion.html"
    ;;
  web)
    serve
    open_url "http://localhost:$PORT/telemetria.html"
    echo "Pagina en http://localhost:$PORT/telemetria.html"
    ;;
  -h|--help|help)
    sed -n '2,10p' "$0" | sed 's/^# \{0,1\}//'
    ;;
  *)
    echo "Opcion desconocida: $1 (usa casos, pesos, simulacion o web)" >&2
    exit 1
    ;;
esac
