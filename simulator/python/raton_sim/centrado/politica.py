import math


def centring(offset, gain, deadband, limit):
    if offset is None or abs(offset) <= deadband:
        return 0.0
    return max(-limit, min(limit, gain * (offset - math.copysign(deadband, offset))))
