import math


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
