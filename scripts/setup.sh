#!/usr/bin/env bash
# Instala todo lo necesario para trabajar con el proyecto (macOS o Linux) y deja el entorno listo.
#
#   scripts/setup.sh                 todo: herramientas del sistema, Python, web, nucleo C++ y firmware
#   scripts/setup.sh --sin-firmware  sin la cadena de PlatformIO (mas rapido; p. ej. en CI)
#   scripts/setup.sh --sin-sistema   no instala paquetes del sistema (solo comprueba que existan)
#
# Crea .venv (Python con numpy, pytest, ruff y PlatformIO), instala las dependencias de web/,
# compila el nucleo y la biblioteca nativa del simulador en build/ y descarga la cadena del firmware.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

FIRMWARE=1
SYSTEM=1
for arg in "$@"; do
  case "$arg" in
    --sin-firmware) FIRMWARE=0 ;;
    --sin-sistema) SYSTEM=0 ;;
    -h|--help) sed -n '2,10p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
    *) echo "Opcion desconocida: $arg" >&2; exit 1 ;;
  esac
done

paso() { printf '\n\033[1m==> %s\033[0m\n' "$*"; }
hay() { command -v "$1" >/dev/null 2>&1; }

# ---------------------------------------------------------------- herramientas del sistema
NODE_MIN=20
node_ok() { hay node && hay npm && [ "$(node -p 'process.versions.node.split(".")[0]')" -ge "$NODE_MIN" ]; }
# Python >= 3.11 (lo pide el simulador); devuelve el primero que sirva.
buscar_python() {
  for p in python3.13 python3.12 python3.11 python3; do
    if hay "$p" && "$p" -c 'import sys; sys.exit(sys.version_info < (3, 11))' 2>/dev/null; then
      command -v "$p"
      return
    fi
  done
}

paso "Herramientas del sistema"
faltan=()
[ -n "$(buscar_python)" ] || faltan+=(python3)
hay cmake || faltan+=(cmake)
(hay c++ || hay clang++ || hay g++) || faltan+=(compilador-c++)
node_ok || faltan+=(node)
hay git || faltan+=(git)
# Debian/Ubuntu separan ensurepip de Python aunque el intérprete ya esté instalado.
if [ "$(uname -s)" = Linux ] && [ -n "$(buscar_python)" ]; then
  "$(buscar_python)" -c 'import venv, ensurepip' 2>/dev/null || faltan+=(python3-venv)
fi

if [ ${#faltan[@]} -gt 0 ]; then
  echo "Faltan: ${faltan[*]}"
  if [ "$SYSTEM" -eq 0 ]; then
    echo "Instalalas y vuelve a ejecutar (o quita --sin-sistema)." >&2
    exit 1
  fi
  case "$(uname -s)" in
    Darwin)
      if ! hay brew; then
        echo "Hace falta Homebrew (https://brew.sh) para instalar: ${faltan[*]}" >&2
        exit 1
      fi
      if [[ " ${faltan[*]} " == *" compilador-c++ "* ]]; then
        echo "Instala las herramientas de Xcode con: xcode-select --install" >&2
        exit 1
      fi
      pkgs=()
      for f in "${faltan[@]}"; do
        case "$f" in
          python3) pkgs+=(python@3.12) ;;
          node) pkgs+=(node) ;;
          *) pkgs+=("$f") ;;
        esac
      done
      brew install "${pkgs[@]}"
      ;;
    Linux)
      if hay apt-get; then
        SUDO=(); if [ "$(id -u)" -ne 0 ]; then SUDO=(sudo -n); fi
        "${SUDO[@]}" apt-get update
        "${SUDO[@]}" env DEBIAN_FRONTEND=noninteractive apt-get install -y python3 python3-venv python3-pip cmake build-essential git curl
        if ! node_ok; then
          curl -fsSL https://deb.nodesource.com/setup_22.x | "${SUDO[@]}" bash -
          "${SUDO[@]}" env DEBIAN_FRONTEND=noninteractive apt-get install -y nodejs
        fi
      else
        echo "Distribucion sin apt-get: instala a mano ${faltan[*]}" >&2
        exit 1
      fi
      ;;
    *)
      echo "Sistema no soportado por este script: instala a mano ${faltan[*]}" >&2
      exit 1
      ;;
  esac
fi
PYTHON="$(buscar_python)"
if [ -z "$PYTHON" ]; then
  echo "Hace falta Python 3.11 o mas nuevo." >&2
  exit 1
fi
echo "python $("$PYTHON" --version | cut -d' ' -f2) · cmake $(cmake --version | head -1 | cut -d' ' -f3) · node $(node --version)"

# ---------------------------------------------------------------- Python
paso "Entorno de Python (.venv)"
[ -d .venv ] || "$PYTHON" -m venv .venv
.venv/bin/python -m pip install --quiet --upgrade pip
if [ "$FIRMWARE" -eq 1 ]; then
  .venv/bin/python -m pip install --quiet -r requirements-dev.txt
else
  grep -v '^platformio' requirements-dev.txt | .venv/bin/python -m pip install --quiet -r /dev/stdin
fi
.venv/bin/python -m pip install --quiet -e simulator/python

# ---------------------------------------------------------------- web
paso "Web (React)"
(cd web && npm ci --no-audit --no-fund)

# CMake descubre los fuentes por funcionalidad en lib/raton_core/src/.
# ---------------------------------------------------------------- nucleo C++
paso "Nucleo C++ y biblioteca nativa del simulador"
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release >/dev/null
cmake --build build --parallel

# ---------------------------------------------------------------- firmware
if [ "$FIRMWARE" -eq 1 ]; then
  paso "Cadena del firmware (PlatformIO)"
  .venv/bin/pio pkg install -e raton
  .venv/bin/pio pkg install -e prueba_hardware
fi

paso "Listo"
echo "Siguiente: scripts/run.sh            (pruebas y web de telemetria)"
echo "           scripts/run.sh ayuda      (todas las ordenes)"
