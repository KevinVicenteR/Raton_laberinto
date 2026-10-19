import math


def braking(speed, maximum, remaining, dt, brake):
    return min(speed + 800 * dt, maximum, max(40, math.sqrt(max(0, 1600 * remaining))) if brake else maximum)


class AdaptiveSpeed:
    """Espejo de velocidad/adaptive_speed.h; distancia desde la ultima maniobra cauta."""

    def __init__(self):
        self.caution_left = 0.0
        self.cap = None

    def caution(self, distance):
        self.caution_left = max(self.caution_left, distance)

    def step(self, fw, maximum, confidence, heading, known, front_clearance, distance, dt):
        self.caution_left = max(0, self.caution_left - max(0, distance))

        def unit(value):
            return max(0, min(1, value))

        trust = min(unit(confidence), unit(1 - abs(heading) / fw.adaptive_heading_deg))
        if front_clearance >= 0:
            trust = min(trust, unit(front_clearance / fw.adaptive_front_mm))
        if self.caution_left > 0:
            trust = 0
        ratio = fw.adaptive_min_ratio + (1 - fw.adaptive_min_ratio) * trust
        if not known:
            ratio = min(ratio, fw.adaptive_unknown_ratio)
        wanted = maximum * ratio
        if self.cap is None:
            self.cap = wanted
        self.cap += max(-fw.accel * dt, min(fw.accel * dt, wanted - self.cap))
        return min(maximum, self.cap)
