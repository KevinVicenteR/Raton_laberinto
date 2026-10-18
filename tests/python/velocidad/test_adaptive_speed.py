"""Contrato numerico compartido con la politica probada en tests/cpp."""

from dataclasses import replace

import pytest
from raton_sim.giros.native import configuration_text
from raton_sim.seguridad.orchestrators import AdaptiveSpeed
from raton_sim.simulacion.simulator import Firmware


@pytest.fixture
def policy():
    return replace(Firmware(), accel=1000, adaptive_min_ratio=0.6, adaptive_unknown_ratio=0.9)


@pytest.mark.parametrize(
    "confidence,heading,known,front,expected",
    [
        (1, 0, True, -1, 300),
        (1, 0, False, -1, 270),
        (0, 0, True, -1, 180),
        (1, 5, True, -1, 180),
        (1, 0, True, 0, 180),
    ],
)
def test_observed_risk_caps_speed(policy, confidence, heading, known, front, expected):
    assert AdaptiveSpeed().step(policy, 300, confidence, heading, known, front, 0, 0.005) == pytest.approx(
        expected
    )


def test_caution_is_distance_based_and_both_transitions_are_smooth(policy):
    speed = AdaptiveSpeed()
    speed.caution(100)
    assert speed.step(policy, 300, 1, 0, True, -1, 50, 0.005) == pytest.approx(180)
    assert speed.caution_left == 50
    assert speed.step(policy, 300, 1, 0, True, -1, 51, 0.005) == pytest.approx(185)
    speed.caution(80)
    assert speed.step(policy, 300, 1, 0, True, -1, -20, 0.005) == pytest.approx(180)
    assert speed.caution_left == 80


@pytest.mark.parametrize("error", [-8, 8])
def test_turn_disturbance_changes_actuator_not_gyro_reference(error):
    import math

    from raton_sim.entrenamiento.benchmark import RandomizedSim, scenario
    from raton_sim.giros.native import firmware_config
    from raton_sim.simulacion import simulator as S

    domain, _ = scenario(192, 10000, "nominal")
    domain["turn"] = error
    case = S.Case("turn", S.comp_maze(), (18, 0), (4, 2), mode=4, fw=firmware_config())
    sim = RandomizedSim(case, domain, 1)
    initial = sim.th
    list(sim.rotate(90))
    assert abs(math.degrees(sim.th - initial) - 90) < 3


def test_side_conversion_uses_configured_angle_not_hidden_mounting_angle():
    import math

    from raton_sim.simulacion import simulator as S

    case = S.Case("angle", S.comp_maze(), (18, 0), (4, 2), robot=S.Robot(side_angle=23))
    sim = S.Sim(case)
    assert sim.side_face(100) == pytest.approx(100 * math.sin(math.radians(31)) - 10)


def test_native_configuration_does_not_silently_use_old_recovery_limits():
    import re

    from raton_sim.giros.native import firmware_config

    text = configuration_text()
    for name, value in (("RUB_MS", "75"), ("MAX_RUBS", "2"), ("SKEW_LIMIT_DEG", "7.5f")):
        text = re.sub(r"(" + name + r"\s*=)[^;]+;", r"\g<1> " + value + ";", text)
    fw = firmware_config(config_text=text)
    assert fw.rub_seconds == 0.075
    assert fw.max_rubs == 2
    assert fw.skew_limit == 7.5
