def turn(error, gain):
    return max(-1.5, min(1.5, (6 / 0.014) * gain * error))
