"""Configura el paquete local y la biblioteca de barrido nativa."""

import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "simulator/python"))
if "RATON_NATIVE_LIB" not in os.environ:
    for name in ("libraton_navigation.dylib", "libraton_navigation.so"):
        path = ROOT / "build" / name
        if path.exists():
            os.environ["RATON_NATIVE_LIB"] = str(path)
            break
