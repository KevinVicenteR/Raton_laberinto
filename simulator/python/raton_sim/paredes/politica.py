import math
from statistics import median


def wall_classifier(samples, threshold, minimum=2):
    valid = [v for v in samples if math.isfinite(v) and v >= 0]
    return len(valid) >= minimum and median(valid) < threshold
