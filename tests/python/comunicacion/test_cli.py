"""Interfaz pública y configuración real del firmware."""

import os
import subprocess
import sys

from raton_sim.giros import native
from raton_sim.simulacion import simulator


def test_cli_help():
    env = dict(os.environ, PYTHONPATH=str(native.ROOT / "simulator/python"), PYTHONDONTWRITEBYTECODE="1")
    for command in ([], ["casos"], ["pistas"], ["evaluar"], ["publicar"], ["pesos"], ["entrenar"]):
        result = subprocess.run(
            [sys.executable, "-m", "raton_sim", *command, "--help"], env=env, capture_output=True, text=True
        )
        assert result.returncode == 0, result.stderr
        assert "usage:" in result.stdout


def test_firmware_path_and_legacy_defaults():
    assert (native.ROOT / "firmware/config.h").is_file()
    fw = native.firmware_config(simulator)
    assert fw.split_deg == 70
    assert fw.split_advance == 75
    assert fw.reverse_short == 25
    assert fw.cfg_side_ahead == 30
    assert fw.turn_timeout == 3


def test_config_is_read_from_firmware(tmp_path, monkeypatch):
    folder = tmp_path / "firmware"
    folder.mkdir()
    text = native.configuration_text()
    (folder / "config.h").write_text(text.replace("SEARCH_SPEED = 100.0f", "SEARCH_SPEED = 123.0f"))
    monkeypatch.setattr(native, "ROOT", tmp_path)
    assert native.firmware_config(simulator).speed == 123
