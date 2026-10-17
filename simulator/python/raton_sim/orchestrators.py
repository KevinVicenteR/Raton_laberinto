"""Deterministic mission/motion policies with optional bounded, stdlib gates."""

import math
from statistics import median


class Gate:
    def __init__(self, weights=None):
        self.weights = weights

    def weights_for(self, features, n):
        try:
            if any(len(row) != len(features) + 1 for row in self.weights):
                return [1.0] * n
            values = [sum(a * b for a, b in zip(row, [1.0] + list(features))) for row in self.weights]
            if len(values) != n or not all(math.isfinite(v) for v in values):
                return [1.0] * n
            return [max(0.5, min(1.5, 1 + math.tanh(v) * 0.5)) for v in values]
        except (TypeError, ValueError, OverflowError):
            return [1.0] * n


def wall_classifier(samples, threshold, minimum=2):
    valid = [v for v in samples if math.isfinite(v) and v >= 0]
    return len(valid) >= minimum and median(valid) < threshold


def centring(offset, gain, deadband, limit):
    if offset is None or abs(offset) <= deadband:
        return 0.0
    return max(-limit, min(limit, gain * (offset - math.copysign(deadband, offset))))


def braking(speed, maximum, remaining, dt, brake):
    return min(speed + 800 * dt, maximum, max(40, math.sqrt(max(0, 1600 * remaining))) if brake else maximum)


def turn(error, gain):
    return max(-1.5, min(1.5, (6 / 0.014) * gain * error))


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
        return centring(offset, fw.kp_wall, fw.deadband, fw.max_wall_deg) * w[0], braking(
            speed, maximum, remaining, dt, brake
        ) * min(1, w[1])

    def safety(self, *args, **kwargs):
        return safety(*args, **kwargs)

    def turn(self, error, gain):
        return turn(error, gain)


class MissionOrchestrator:
    def __init__(self, gate=None):
        self.gate = Gate(gate)
        self.failures = 0
        self.pruned = set()
        self.unreachable = set()
        self.state = "read_start"

    def decide(self, start=False, finished=False, route=False, recover=False):
        # Safety and mission completion cannot be overridden by learned weights.
        self.state = (
            "finish"
            if finished
            else (
                "read_start"
                if start
                else ("recover" if recover else ("shortest_route" if route else "explore_step"))
            )
        )
        self.gate.weights_for([self.failures], 1)
        return self.state

    def choose(self, ordered_options):
        weights = self.gate.weights_for([self.failures], len(ordered_options))
        return min(enumerate(ordered_options), key=lambda pair: (pair[0] + 1) / weights[pair[0]])[1]

    def attempts(self, limit):
        return range(max(1, int(limit)))

    def progress(self, moved, node, limit):
        self.failures = 0 if moved else self.failures + 1
        if self.failures >= limit:
            self.unreachable.add(node)
            return False
        return True

    def prune(self, seen, goal, pos, wall, moves):
        changed = True
        while changed:
            changed = False
            for node in seen - self.pruned - {goal, pos}:
                exits = [
                    (node[0] + dx, node[1] + dy) for k, (dx, dy) in enumerate(moves) if not wall(*node, k)
                ]
                if sum(n not in self.pruned for n in exits) <= 1:
                    self.pruned.add(node)
                    changed = True
        return self.pruned
