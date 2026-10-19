import math

from raton_sim.giros import native as N
from raton_sim.simulacion.modelos import (
    MotionFailure,
)


class GirosSim:
    def prepare_turn(self, degrees):
        if self.c.robot.side_angle != 90 or self.c.controller == "baseline" or not self.c.sweep_guard:
            return
        entry = self.native.turn(self.observation(), self.c.robot, self.c.fw, degrees)
        if entry is None:
            raise MotionFailure("obstaculo observado en el barrido del giro")
        start = self.travelled
        if entry < 0:
            yield from self.back(-entry)
        elif entry > 0:
            yield from self.drive(entry, 0)
        if abs(self.travelled - start - entry) > 3:
            raise MotionFailure("ajuste de entrada incompleto")

    @N.instrument
    def rotate(self, deg):
        if self.c.fw.profiled_turn:
            yield from self.rotate_profiled(deg)
            return
        self.turns += 1
        target = self.th + math.radians(deg) + math.radians(self.rnd.gauss(0, 1.5))
        t0, blocked_since, retries = self.t, None, 0
        while (
            abs(target - self.th) > math.radians(0.5 if self.c.controller == "baseline" else 3)
            and self.t - t0 < self.c.fw.turn_timeout
        ):
            yield
            self.tick()
            step = math.copysign(min(abs(target - self.th), math.radians(180) * self.DT), target - self.th)
            if self.try_move(self.x, self.y, self.th + step):
                blocked_since = None
            else:
                blocked_since = blocked_since if blocked_since is not None else self.t
                # Giro bloqueado (una esquina toca la pared): atras 15 mm y seguir girando.
                if (
                    self.c.fw.unstick
                    and self.c.controller == "baseline"
                    and self.t - blocked_since > 0.3
                    and retries < 3
                ):
                    retries += 1
                    blocked_since = None
                    yield from self.back(15)

        if abs(target - self.th) > math.radians(3):
            raise MotionFailure("giro incompleto")

    def rotate_profiled(self, degrees):
        """Misma consigna/PWM del firmware; N20 aproximado con friccion e inercia.

        900 dps/PWM sobre PWM_MIN_RUN y tau=50 ms son supuestos, no una
        identificacion del robot. La aceleracion disponible se limita a 900 dps2.
        """
        self.turns += 1
        fw = self.c.fw
        target = math.degrees(self.th) + degrees
        start, rate, settled = self.t, 0.0, 0
        while self.t - start < fw.turn_timeout:
            yield
            self.tick()
            error = target - math.degrees(self.th)
            if abs(error) < 3:
                rate -= math.copysign(min(abs(rate), 2 * fw.turn_accel * self.DT), rate)
                settled = settled + 1 if abs(rate) < 8 else 0
                if settled >= 3:
                    break
            else:
                settled = 0
                sign = math.copysign(1, error)
                desired = sign * min(fw.turn_rate, math.sqrt(2 * fw.turn_accel * max(0, abs(error) - 1)))
                pwm = (
                    desired / 900
                    + sign * fw.pwm_min_run * min(1, abs(error) / 6)
                    + fw.kd_turn * (desired - rate)
                )
                if fw.legacy_turn:
                    if abs(error) < 5 and abs(rate) < 20 and self.t - start > 0.3:
                        break
                    pwm = 0.006 * error - fw.kd_turn * rate
                pwm = max(-fw.turn_pwm_max, min(fw.turn_pwm_max, pwm))
                if (
                    (fw.legacy_turn or (self.t - start < 0.2 and abs(error) > 12))
                    and abs(rate) < 20
                    and abs(pwm) < fw.turn_pwm_min
                ):
                    pwm = sign * fw.turn_pwm_min
                equilibrium = self.c.turn_motor_gain * math.copysign(
                    max(0, abs(pwm) - fw.pwm_min_run) * 900, pwm
                )
                available = fw.turn_accel * self.c.turn_motor_gain
                acceleration = max(-available, min(available, (equilibrium - rate) / self.c.turn_tau_s))
                rate += acceleration * self.DT
            self.try_move(self.x, self.y, self.th + math.radians(rate) * self.DT)
        stop_until = self.t + fw.stop_seconds
        while self.t < stop_until:
            yield
            self.tick()
            rate -= math.copysign(min(abs(rate), 2 * fw.turn_accel * self.DT), rate)
            self.try_move(self.x, self.y, self.th + math.radians(rate) * self.DT)
        if abs(target - math.degrees(self.th)) >= (10 if fw.legacy_turn else 3):
            raise MotionFailure("giro incompleto")

    def rotate_precise_to(self, target):
        """Coarse move retains actuator error; gyro feedback then corrects the observed error.

        Only gyro heading enters the fine loop. Collision tests and sensor noise remain active.
        This models an additional control loop, not a quieter gyro or a smaller robot.
        """

        def error():
            return (target - self.th + math.pi) % (2 * math.pi) - math.pi

        yield from self.rotate(math.degrees(error()))
        start = self.t
        while abs(error()) > math.radians(0.5) and self.t - start < 1.0:
            yield
            self.tick()
            step = math.copysign(min(abs(error()), math.radians(30) * self.DT), error())
            # Fine-stage actuator disturbance; feedback does not get access to any walls.
            step += math.radians(self.rnd.gauss(0, 1.5)) * self.DT
            self.try_move(self.x, self.y, self.th + step)
        if abs(error()) > math.radians(0.5):
            raise MotionFailure("refinamiento angular incompleto")

    @N.instrument
    def safe_turn(self, deg):
        """Variantes de giro segun fw.turn_mode."""
        fw = self.c.fw
        if fw.turn_mode == "dos_tiempos":
            if abs(deg) == 90:
                yield from self.prepare_turn(deg)
                sgn = 1 if deg > 0 else -1
                heading = self.th
                if fw.precision_turns:
                    yield from self.rotate_precise_to(heading + math.radians(sgn * fw.split_deg))
                else:
                    yield from self.rotate(sgn * fw.split_deg)
                yield from self.drive(fw.split_advance, 0, corner=fw.corner_leg)
                if fw.precision_turns:
                    yield from self.rotate_precise_to(heading + math.radians(sgn * 90))
                else:
                    yield from self.rotate(sgn * (90 - fw.split_deg))
                return
            if abs(deg) == 180:
                side = yield from self.back_out(3 * self.pitch)
                if side:
                    # El hueco queda detras del laser: entrar con el giro en dos tiempos hacia ese lado.
                    yield from self.safe_turn(90 * side)
                    return
        if fw.turn_mode == "seguro":
            for _ in range(12):
                if not self._turn_hits(deg):
                    break
                yield from self.back(8)
        yield from self.rotate(deg)

    def _turn_hits(self, deg):
        for k in range(1, 7):
            if self.collides(self.x, self.y, self.th + math.radians(deg) * k / 6):
                return True
        return False

    @N.instrument
    def turn_corner_track(self, deg):
        self.explore_v = 0.0
        yield from self.prepare_turn(deg)
        target = (self.grid_th if self.grid_th is not None else self.th) + math.radians(deg)
        yield from self.cell_rotate_to(target)
        self.grid_th = target
        self.pending = self.along = 0.0

    def cell_rotate_to(self, target):
        yield from self._cell_rotate_to(target)
        yield from self.wait_fresh_front()
        if hasattr(self, "adaptive_speed"):
            self.adaptive_speed.caution(self.c.fw.adaptive_caution_mm)

    def _cell_rotate_to(self, target):
        if self.c.fw.profiled_turn:
            threshold = 5 if self.c.fw.legacy_turn else 3
            attempts = self.c.fw.turn_attempts
            for _ in range(attempts):
                try:
                    yield from self.rotate(math.degrees(target - self.th))
                except MotionFailure:
                    pass
                error = abs(math.degrees(target - self.th))
                if error <= threshold:
                    break
            if error >= (10 if self.c.fw.legacy_turn else 3):
                raise MotionFailure("giro incompleto")
            return
        if not self.c.fw.cell_turn_retry:
            # Legacy acceptance: a stopped turn within twenty degrees proceeds.
            try:
                yield from self.rotate(math.degrees(target - self.th))
            except MotionFailure:
                if abs(math.degrees(target - self.th)) > 20:
                    raise
            return
        # Keep the established fine feedback, retry a stopped coarse/fine turn twice.
        for attempt in self.mission.attempts(self.c.fw.turn_attempts):
            try:
                yield from self.rotate_precise_to(target)
            except MotionFailure:
                pass
            error = abs(math.degrees((target - self.th + math.pi) % (2 * math.pi) - math.pi))
            if error <= 5:
                break
        error = abs(math.degrees((target - self.th + math.pi) % (2 * math.pi) - math.pi))
        if error > 10:
            raise MotionFailure("giro incompleto")
        if error > 3:
            try:
                yield from self.rotate_precise_to(target)
            except MotionFailure:
                if abs(math.degrees(target - self.th)) > 10:
                    raise
