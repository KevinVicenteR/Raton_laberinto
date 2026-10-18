"""Persistencia del modelo con el formato binario portable de la placa."""

import math
import tempfile
from pathlib import Path

import pytest
from raton_sim.mapa import map_storage as M


def test_restart_preserves_geometry_and_corruption_is_rejected():
    walls = {(20, 20): 1, (20, 21): 4}
    args = walls, {(20, 20)}, 192, 204, math.sin(math.radians(31)), 50
    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory) / "track.bin"
        M.save(path, *args)
        restored, seen, geometry = M.load(path)
        assert restored[20, 20] == 1 and restored[20, 21] == 4
        assert seen == {(20, 20)}
        assert geometry[:2] == (192, 204)
        for offset in (3, 8, 20, 30, len(path.read_bytes()) - 1):
            corrupt = bytearray(path.read_bytes())
            corrupt[offset] ^= 1
            with pytest.raises(ValueError):
                M.decode(corrupt)
        path.unlink()
        with pytest.raises(FileNotFoundError):
            M.load(path)


def test_invalid_map_cannot_replace_existing_record(tmp_path):
    path = tmp_path / "track.bin"
    M.save(path, {}, set(), 192, 204, 0.515, 50)
    before = path.read_bytes()
    with pytest.raises(ValueError):
        M.save(path, {(20, 20): 1}, set(), 192, 204, 0.515, 50)
    assert path.read_bytes() == before


def test_mode_six_returns_saves_and_new_simulator_loads_route(tmp_path):
    from raton_sim.giros.native import firmware_config
    from raton_sim.simulacion import simulator as S

    path = str(tmp_path / "flash.bin")
    learned = S.Sim(
        S.Case(
            "learn",
            S.M_L,
            (2, 0),
            (0, 2),
            mode=6,
            learned_path=path,
            fw=firmware_config(S),
            controller="corrected",
            seconds=60,
        )
    ).run()
    assert learned.mapping_complete and learned.failure is None
    assert learned.odo_cell() == (2, 0)
    assert abs(math.degrees(learned.th) - 90) < 3
    assert Path(path).exists()
    restarted = S.Sim(
        S.Case(
            "route",
            S.M_L,
            (2, 0),
            (0, 2),
            mode=3,
            use_goal=True,
            learned_path=path,
            fw=firmware_config(S),
            controller="corrected",
            seconds=60,
        )
    ).run()
    assert any(event[1] == "MAPA_CARGADO" for event in restarted.events)
    assert restarted.reached is not None
    assert restarted.collisions == 0
