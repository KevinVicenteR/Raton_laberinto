#!/usr/bin/env bash
# Desde la raiz: bash ./web.sh
# Opciones de Vite: bash ./web.sh --port 5174
set -euo pipefail
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$PROJECT_DIR/web"

if ! command -v node >/dev/null 2>&1 || ! command -v npm >/dev/null 2>&1; then
  echo "Faltan Node.js o npm. Instala Node.js y vuelve a abrir la terminal." >&2
  exit 1
fi
if [[ ! -d node_modules ]]; then
  echo "Instalando dependencias de la web..."
  npm ci
fi

echo "Iniciando la web. Ctrl+C para detenerla."
# Vite muestra la direccion real y abre el navegador predeterminado.
exec npm run dev -- --open "$@"
