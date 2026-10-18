"""La velocidad no puede compensar restricciones incumplidas al elegir el firmware."""

import json

import pytest
from raton_sim.entrenamiento.robust_motion import select_checkpoint


def test_selection_rejects_fast_unsafe_or_full_speed_unknown_cells(tmp_path):
    initial = dict(
        params={"adaptive_unknown_ratio": 0.9}, training_score=200, validation_score=200, generation=-1
    )
    curve = [
        dict(
            generation=0,
            candidates=[
                {"adaptive_unknown_ratio": 0.9},
                {"adaptive_unknown_ratio": 1},
                {"adaptive_unknown_ratio": 0.95},
            ],
            training_scores=[1000001, 90, 100],
            validation=[
                dict(candidate=0, score=10),
                dict(candidate=1, score=20),
                dict(candidate=2, score=100),
            ],
        )
    ]
    (tmp_path / "initial.json").write_text(json.dumps(initial))
    (tmp_path / "curve.json").write_text(json.dumps(curve))
    selected = select_checkpoint(tmp_path)
    assert selected["candidate"] == 2
    assert selected["validation_score"] == 100
    assert json.loads((tmp_path / "selected.json").read_text())["feasible"] is True


def test_no_admissible_checkpoint_does_not_write_a_winner(tmp_path):
    (tmp_path / "initial.json").write_text(
        json.dumps(
            dict(
                params={"adaptive_unknown_ratio": 0.9},
                training_score=1000001,
                validation_score=1000001,
                generation=-1,
            )
        )
    )
    (tmp_path / "curve.json").write_text("[]")
    with pytest.raises(RuntimeError, match="Ningun checkpoint"):
        select_checkpoint(tmp_path)
    assert not (tmp_path / "selected.json").exists()
