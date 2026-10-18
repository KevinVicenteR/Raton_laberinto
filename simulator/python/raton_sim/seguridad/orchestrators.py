"""Deterministic mission/motion policies with optional bounded, stdlib gates."""

import math

from raton_sim.centrado.politica import centring as centring
from raton_sim.entrenamiento.politica import Gate as Gate
from raton_sim.exploracion.politica import MissionOrchestrator as MissionOrchestrator
from raton_sim.giros.politica import turn as turn
from raton_sim.paredes.politica import wall_classifier as wall_classifier
from raton_sim.velocidad.politica import AdaptiveSpeed as AdaptiveSpeed
from raton_sim.velocidad.politica import braking as braking


def safety(scrape_seconds, rub_seconds, recoveries, max_rubs, stalled=False):
    return (
        "veto"
        if stalled or recoveries >= max_rubs
        else ("recover" if scrape_seconds >= rub_seconds else "run")
    )


class MotionOrchestrator:
    def __init__(self, gate=None):
        self.gate = Gate(gate)
        self.front_previous = None
        self.front_count = 0
        self.front_sample = -1

    def classify(self, samples, threshold, minimum=2):
        return wall_classifier(samples, threshold, max(2, minimum))

    def confirmed_front(self, value, sample):
        if sample != self.front_sample:
            self.front_count = (
                self.front_count + 1
                if value >= 0 and self.front_previous is not None and abs(value - self.front_previous) <= 30
                else (1 if value >= 0 else 0)
            )
            self.front_previous = value if value >= 0 else None
            self.front_sample = sample
        return self.front_count >= 2

    def tick(self, offset, fw, speed, maximum, remaining, dt, brake):
        w = self.gate.weights_for([remaining / 300, speed / 300], 2)
        limit = max(40, math.sqrt(max(0, 2 * fw.accel * remaining))) if brake else maximum
        velocity = min(speed + fw.accel * dt, maximum, limit)
        return centring(offset, fw.kp_wall, fw.deadband, fw.max_wall_deg) * w[0], velocity * min(1, w[1])

    def safety(self, *args, **kwargs):
        return safety(*args, **kwargs)

    def turn(self, error, gain):
        return turn(error, gain)
