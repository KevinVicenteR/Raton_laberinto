import math

import pytest
from raton_sim.giros.native import firmware_config
from raton_sim.simulacion import simulator as S


@pytest.mark.parametrize("gain,tau", [(0.75, 0.08), (1, 0.05), (1.2, 0.03)])
@pytest.mark.parametrize("angle", [90, -90, 180, 5, -5])
def test_profile_stops_within_three_degrees_without_contact(gain, tau, angle):
    case = S.Case(
        "turn",
        S.random_maze(3, 3, seed=1, start=(1, 1)),
        (1, 1),
        (0, 0),
        corridor=168,
        fw=firmware_config(S),
        controller="corrected",
        turn_motor_gain=gain,
        turn_tau_s=tau,
    )
    sim = S.Sim(case)
    initial = sim.th
    list(sim.rotate(angle))
    assert abs(angle - math.degrees(sim.th - initial)) < 3
    assert sim.t < case.fw.turn_timeout
    assert sim.collisions == 0
