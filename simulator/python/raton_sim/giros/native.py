"""Enlace nativo de barrido: solo recibe observaciones y geometría configurada."""

import ctypes
import math
import os
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]


def configuration_text(path=None):
    """Expande los params.h locales sin duplicar constantes en el simulador."""
    path = Path(path) if path is not None else ROOT / "firmware/config.h"
    visited = set()

    def read(source):
        source = source.resolve()
        if source in visited:
            return ""
        visited.add(source)
        text = source.read_text()

        def include(match):
            target = ROOT / "firmware" / match.group(1)
            return read(target)

        return re.sub(r'^#include "([^"\n]+)"', include, text, flags=re.MULTILINE)

    return read(path)


def firmware_config(sim_module=None, *, config_text=None):
    """Lee el firmware actual conservando los valores retirados que usa el simulador."""
    from dataclasses import replace

    if sim_module is None:
        from raton_sim.simulacion import simulator as sim_module

    text = config_text if config_text is not None else configuration_text()

    def number(name, default=None):
        # Resuelve expresiones aritméticas de geometría sin ejecutar el archivo de configuración.
        import ast
        import operator

        declarations = dict(
            re.findall(r"const\s+(?:float|int|uint8_t|uint16_t|uint32_t)\s+(\w+)\s*=\s*([^;]+);", text)
        )
        ops = {
            ast.Add: operator.add,
            ast.Sub: operator.sub,
            ast.Mult: operator.mul,
            ast.Div: operator.truediv,
        }

        def value(node, active):
            if isinstance(node, ast.Constant) and type(node.value) in (int, float):
                return node.value
            if isinstance(node, ast.Name):
                return resolve(node.id, active)
            if isinstance(node, ast.UnaryOp) and isinstance(node.op, ast.USub):
                return -value(node.operand, active)
            if isinstance(node, ast.BinOp) and type(node.op) in ops:
                return ops[type(node.op)](value(node.left, active), value(node.right, active))
            raise ValueError("expresión de constante no admitida")

        def resolve(key, active):
            if key in active or key not in declarations:
                raise ValueError(key)
            expression = re.sub(r"(?<=\d)f\b", "", declarations[key])
            return value(ast.parse(expression, mode="eval").body, active | {key})

        try:
            return float(resolve(name, set()))
        except (ValueError, SyntaxError, ZeroDivisionError):
            if default is not None:
                return default
            raise ValueError(f"configuración no numérica: {name}")

    mapping = dict(
        adaptive_min_ratio="ADAPT_MIN_RATIO",
        adaptive_unknown_ratio="ADAPT_UNKNOWN_RATIO",
        adaptive_heading_deg="ADAPT_HEADING_DEG",
        adaptive_caution_mm="ADAPT_CAUTION_MM",
        adaptive_front_mm="ADAPT_FRONT_MM",
        accel="ACCEL",
        yaw_smooth="YAW_TARGET_SMOOTH",
        kd_yaw="KD_YAW",
        wall_rate_damp="WALL_RATE_DAMP",
        turn_rate="TURN_RATE_DPS",
        turn_accel="TURN_ACCEL_DPS2",
        turn_pwm_max="TURN_PWM_MAX",
        turn_pwm_min="TURN_PWM_MIN",
        pwm_min_run="PWM_MIN_RUN",
        kd_turn="KD_TURN",
        fresh_samples="POST_TURN_SAMPLES",
        calibration_samples="CALIBRATION_SAMPLES",
        front_stable_samples="FRONT_STABLE_SAMPLES",
        speed="SEARCH_SPEED",
        side_wall="SIDE_WALL_MM",
        front_wall="FRONT_WALL_MM",
        cfg_side_ahead="SIDE_SENSOR_AHEAD_MM",
        cfg_front_ahead="FRONT_SENSOR_AHEAD_MM",
        robot_width_cfg="ROBOT_WIDTH_MM",
        kp_wall="KP_WALL",
        deadband="LATERAL_DEADBAND_MM",
        max_wall_deg="MAX_WALL_CORRECTION_DEG",
        side_crash="SIDE_CRASH_MM",
        split_deg="TURN_SPLIT_DEG",
        split_advance="TURN_SPLIT_ADVANCE_MM",
        close_mm="SIDE_CLOSE_MM",
        comp_speed="COMP_SPEED",
        reverse_short="REVERSE_SHORT_MM",
        w_goal="W_GOAL",
        w_exit="W_EXIT",
        front_margin="FRONT_MARGIN_MM",
    )
    extra = (
        {
            "explore_speed": number("EXPLORE_SPEED", 150),
            "kp_yaw": number("KP_YAW", 0.014),
            "turn_timeout": number("TURN_TIMEOUT_MS") / 1000,
        }
        if hasattr(sim_module.Firmware(), "turn_timeout")
        else {}
    )
    extra.update(
        rub_seconds=number("RUB_MS", 100) / 1000,
        max_rubs=int(number("MAX_RUBS", 3)),
        skew_limit=number("SKEW_LIMIT_DEG", 10),
        max_no_progress=int(number("MAX_NO_PROGRESS", 3)),
        turn_attempts=int(number("TURN_ATTEMPTS", 3)),
    )
    for key in ("cell_turn_retry", "cell_scrape_recovery"):
        if hasattr(sim_module.Firmware(), key):
            extra[key] = os.environ.get("RATON_" + key.upper(), "1") != "0"
    length = number("BODY_LENGTH_MM", 80)
    width = number("BODY_WIDTH_MM", 80)
    axle = number("AXLE_BEHIND_CENTER_MM", 0)
    geometry = replace(
        sim_module.Robot(),
        length_front=length / 2 + axle,
        length_back=length / 2 - axle,
        width=width,
        side_ahead=number("SIDE_SENSOR_AHEAD_MM", 30),
        side_inset=width / 2 - number("SIDE_SENSOR_LATERAL_MM", 30),
        side_angle=31.0,
        front_ahead=number("FRONT_SENSOR_AHEAD_MM", 40),
        deadzone=50.0,
    )
    values = {
        k: number(
            v,
            {
                "adaptive_min_ratio": 1,
                "adaptive_unknown_ratio": 1,
                "adaptive_heading_deg": 5,
                "adaptive_caution_mm": 0,
                "adaptive_front_mm": 120,
                "wall_rate_damp": 0,
                "turn_rate": 180,
                "turn_accel": 900,
                "fresh_samples": 8,
                "calibration_samples": 15,
                "front_stable_samples": 9,
                "cfg_side_ahead": 30,
                "cfg_front_ahead": 40,
                "robot_width_cfg": width,
                "split_deg": 70,
                "split_advance": 75,
                "reverse_short": 25,
            }.get(k),
        )
        for k, v in mapping.items()
    }
    front_stop = float(os.environ.get("RATON_CELL_FRONT_STOP_MM", 0))
    values["front_margin"] = max(values["front_margin"], front_stop - geometry.deadzone)
    return replace(
        sim_module.Firmware(),
        geometry=geometry,
        profiled_turn=True,
        stop_seconds=(20 + number("SETTLE_MS")) / 1000,
        **extra,
        **values,
        cell_side_wall=float(
            os.environ.get("RATON_CELL_SIDE_WALL_MM", max(96, number("CELL_SIDE_WALL_MM", 70)))
        ),
        side_look_ahead=float(
            os.environ.get("RATON_SIDE_LOOK_AHEAD_MM", 30 + (192 / 2 - 30) / math.tan(math.radians(31)))
        ),
        corner_front=30,
        turn_mode="sitio",
        unstick=True,
        weighted=True,
        edge_fix=False,
        w=tuple(number(n) for n in ("W_FREE", "W_NOVEL", "W_TURN", "B_STRAIGHT")),
    )


class NativeNavigation:
    def __init__(self, path=None):
        path = path or os.environ.get("RATON_NATIVE_LIB")
        if not path:
            path = next(
                (
                    p
                    for p in (
                        ROOT / "build/libraton_navigation.dylib",
                        ROOT / "build/libraton_navigation.so",
                        ROOT / "build/raton_navigation.dll",
                    )
                    if p.exists()
                ),
                None,
            )
        if not path:
            raise RuntimeError("Indica la biblioteca raton_navigation en RATON_NATIVE_LIB")
        self.lib = ctypes.CDLL(str(path))
        fp = ctypes.POINTER(ctypes.c_float)
        self.lib.raton_turn.argtypes = [fp, fp, ctypes.c_float, ctypes.c_float, ctypes.c_float]
        self.lib.raton_turn.restype = ctypes.c_float

    @staticmethod
    def array(values):
        return (ctypes.c_float * len(values))(*values)

    def turn(self, observation, robot, fw, degrees):
        if len(observation) != 3 or any(len(s) != 4 for s in observation):
            return None
        values = [v for s in observation for v in s]
        body = (
            robot.length_front,
            robot.length_back,
            robot.width,
            fw.cfg_side_ahead,
            fw.cfg_front_ahead,
            fw.robot_width_cfg,
        )
        out = self.lib.raton_turn(
            self.array(values), self.array(body), degrees, fw.split_deg, fw.split_advance
        )
        return None if out == 10000 or not math.isfinite(out) else float(out)


def instrument(method):
    """Registra resultados de maniobras y propaga los fallos al llamador."""
    from functools import wraps

    @wraps(method)
    def wrapped(self, *args, **kwargs):
        kind = method.__name__
        start, distance, heading, contacts = self.t, self.travelled, self.th, self.collisions
        requested = float(args[0]) if args else 0
        if kind == "run_cells":
            requested *= self.cell_mm if self.c.mode == 3 else self.pitch
        reason, completed, result = "complete", True, None
        try:
            result = yield from method(self, *args, **kwargs)
            if kind == "drive" and result != "dist":
                completed, reason = False, str(result)
            if kind == "back_out" and not result:
                completed, reason = False, "no_opening"
            if kind == "back" and abs(self.travelled - distance) < requested - 1:
                completed, reason = False, "interrupted"
            if kind == "run_cells" and (result[0] < args[0] or result[1]):
                completed, reason = False, "interrupted"
            return result
        except GeneratorExit:
            completed, reason = False, "goal_reached" if self.reached is not None else "simulation_closed"
            raise
        except MotionFailureBase as exc:
            completed, reason = False, str(exc)
            raise
        finally:
            actual = self.travelled - distance
            delta = math.degrees(self.th - heading)
            angular_error = requested - delta if kind in ("safe_turn", "turn_corner_track", "rotate") else 0
            record = dict(
                t=start,
                end=self.t,
                kind=kind,
                requested=requested,
                actual_mm=actual,
                angular_error=angular_error,
                completed=completed,
                reason=reason,
                contacts=self.collisions - contacts,
                observation=self.observation(),
                pending=self.pending,
            )
            self.motions.append(record)
            if len(self.motions) > 20000:
                raise RuntimeError("se alcanzó el límite del registro de maniobras")

    return wrapped


class MotionFailureBase(RuntimeError):
    pass
