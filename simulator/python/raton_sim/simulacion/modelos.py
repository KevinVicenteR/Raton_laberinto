import os
from dataclasses import dataclass, field

from raton_sim.giros import native as N


@dataclass
class Robot:
    length_front: float = 40.0
    length_back: float = 40.0
    width: float = 80.0
    side_ahead: float = 30.0
    side_inset: float = 10.0
    side_angle: float = 31.0
    front_ahead: float = 40.0
    deadzone: float = 50.0

    @classmethod
    def preset(cls, name="real"):
        if name == "antiguo":
            return cls(83, 17, 92, 56, 0, 90, 33, 50)
        if name != "real":
            raise ValueError(name)
        return cls()


@dataclass
class Sensor:
    max_mm: float = 300.0  # LASER_MAX_MM
    min_mm: float = 30.0  # por debajo, el VL53L0X da basura o nada
    noise: float = 3.0  # desviacion tipica (mm)
    dropout: float = 0.02  # probabilidad de lectura perdida
    cone_deg: float = 0.0  # identified effective ray, not nearest of an ideal cone
    distance_dropout: bool = True  # wood: 2% below 100, 30% from 200 mm
    mounting_error: float = 5.0  # independent fixed error per side, degrees
    latency_ms: float = 0.0  # additional simulated delivery latency; not a measured calibration
    blind_left: bool = False  # laser izquierdo roto (nunca mide)


@dataclass
class Firmware:
    adaptive_min_ratio: float = 1.0
    adaptive_unknown_ratio: float = 1.0
    adaptive_heading_deg: float = 5.0
    adaptive_caution_mm: float = 0.0
    adaptive_front_mm: float = 120.0
    geometry: object = None
    accel: float = 800.0
    yaw_smooth: float = 0.1
    kd_yaw: float = 0.0006
    wall_rate_damp: float = 0.0
    turn_rate: float = 180.0
    turn_accel: float = 900.0
    turn_pwm_max: float = 0.4
    turn_pwm_min: float = 0.28
    pwm_min_run: float = 0.18
    kd_turn: float = 0.001
    legacy_turn: bool = False
    profiled_turn: bool = False
    calibration_samples: int = 15
    front_stable_samples: int = 9
    fresh_samples: int = 8
    stop_seconds: float = 0.1
    cell_side_wall: float = 96.0
    side_look_ahead: float = 139.84
    explore_speed: float = 150.0
    front_factor: float = 0.35
    kp_yaw: float = 0.014  # firmware scale; linearized plant 6 rad/s/rad at .014
    speed: float = 100.0
    side_wall: float = 80.0  # SIDE_WALL_MM = SIDE_OPEN_MM
    front_wall: float = 140.0  # FRONT_WALL_MM
    front_margin: float = 15.0
    cfg_side_ahead: float = 30.0  # lo que cree el firmware (SIDE_SENSOR_AHEAD_MM)
    cfg_front_ahead: float = 40.0
    robot_width_cfg: float = 80.0
    kp_wall: float = 0.15
    max_wall_deg: float = 4.0
    deadband: float = 8.0
    side_crash: float = 8.0
    raw_survey: bool = False  # experimental: classify stationary walls from acquired samples
    survey_samples: int = 5  # fresh stationary observations per new cell
    goal_grid: bool = False  # experimental mode 5 with a goal uses persistent cell exploration
    cell_min_samples: int = 2  # valid side samples required in the cell window
    rub_seconds: float = 0.1
    skew_limit: float = 10.0
    max_rubs: int = 3
    max_no_progress: int = 3
    turn_attempts: int = 3
    cell_turn_retry: bool = True
    cell_scrape_recovery: bool = True
    precision_turns: bool = False  # experimental gyro refinement before each diagonal
    turn_timeout: float = 4.0  # legacy baseline; main bank imports TURN_TIMEOUT_MS
    turn_mode: str = "sitio"  # "sitio" = gira sobre el eje; "seguro" = retrocede antes;
    # "dos_tiempos" = gira 70, avanza, gira 20 (y sale marcha atras de callejones)
    split_deg: float = 70.0
    split_advance: float = 75.0
    close_mm: float = 60.0  # -1 tras una lectura por debajo de esto = demasiado cerca
    comp_speed: float = 250.0  # modo 4: velocidad de los tramos
    corner_front: float = 65.0  # CORNER_FRONT_MM: parar con el laser a esto de la pared para poder girar
    turn_lateral: float = (
        -10.0
    )  # TURN_LATERAL_MM: eje al acabar el tramo del giro, respecto al centro del pasillo nuevo
    adaptive_split: bool = False  # angulo del primer tiempo segun donde esta el eje (si no, split_deg)
    edge_fix: bool = False  # corregir la posicion con los finales de pared laterales
    edge_bias_open: float = 12.0  # EDGE_OPEN_MM: laser pasado el borde de celda al ver que se acaba la pared
    edge_bias_wall: float = 12.0  # EDGE_WALL_MM: laser antes del borde al ver que empieza la pared
    turn_lat_comp: bool = False  # alargar/acortar el tramo del giro segun el descentrado lateral
    robust_lateral: bool = False  # centrado: si L+R no cuadran con el pasillo, solo la lectura corta
    corner_leg: bool = True  # tramo del giro sin roces ni centrado
    fused_stop: bool = False  # parada ante pared: mitad encoders, mitad laser
    align_front: bool = False  # ajustar con el frontal antes de girar si hay pared delante
    reverse_short: float = 25.0  # REVERSE_SHORT_MM: marcha atras, parar antes para que la cola quepa al girar
    w_goal: float = 0.6  # modo 5: peso de la direccion hacia la meta
    w_exit: float = 0.7  # modo 5: peso de las salidas ya usadas (Tremaux)
    unstick: bool = False  # desatasco: atrás y enderezarse si se atasca con el frente libre
    weighted: bool = False  # decision ponderada en los cruces (como el firmware)
    w: tuple = (1.0, 0.8, 0.3, 0.2)  # espacio, novedad, giro, sesgo recto


@dataclass
class Case:
    name: str
    maze: list
    start: tuple  # celda (x, y) contando desde abajo a la izquierda
    goal: tuple
    corridor: float = field(default_factory=lambda: float(os.environ.get("RATON_CORRIDOR_MM", 192)))  # mm
    wall: float = 12.0  # grosor de pared
    mode: int = 5
    off_x: float = 0.0  # descentrado inicial (mm)
    off_deg: float = 0.0  # torcido inicial (grados)
    turn_motor_gain: float = 1.0  # margen físico N20, separado de la consigna
    turn_tau_s: float = 0.05
    drift_dps: float = 0.0  # deriva del rumbo (rueda mas floja), grados/s
    robot: Robot = field(default_factory=Robot)
    sensor: Sensor = field(default_factory=Sensor)
    fw: Firmware = field(default_factory=Firmware)
    hand: tuple = None  # obstaculo movil: (x, y, desde_s, hasta_s) en mm
    seconds: float = 90.0
    known: list = None  # mapa que cree conocer el raton (modo 4); None = el real
    controller: str = "baseline"  # baseline, corrected
    sweep_guard: bool = True
    learned_path: str | None = None  # archivo que emula la flash entre ejecuciones
    trace: bool = False
    use_goal: bool = False  # modo 5 con meta (direccion hacia la meta + memoria de cruces)


# ---------------------------------------------------------------- geometria


def seg_hit(ox, oy, dx, dy, ax, ay, bx, by):
    """Distancia del rayo (o + t d) al segmento ab, o None."""
    ex, ey = bx - ax, by - ay
    den = dx * ey - dy * ex
    if abs(den) < 1e-9:
        return None
    t = ((ax - ox) * ey - (ay - oy) * ex) / den
    u = ((ax - ox) * dy - (ay - oy) * dx) / den
    if t >= 0 and 0 <= u <= 1:
        return t
    return None


def segs_cross(p1, p2, p3, p4):
    def orient(a, b, c):
        return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])

    d1, d2 = orient(p3, p4, p1), orient(p3, p4, p2)
    d3, d4 = orient(p1, p2, p3), orient(p1, p2, p4)
    return (d1 * d2 < 0) and (d3 * d4 < 0)


def build_walls(case):
    """Segmentos de pared (cada pared es un rectangulo de grosor `wall`) a partir del dibujo."""
    rows = len(case.maze) // 2
    cols = (len(case.maze[0]) - 1) // 4
    pitch = case.corridor + case.wall
    h = case.wall / 2
    rects = []
    for r in range(rows + 1):
        line = case.maze[2 * r]
        y = (rows - r) * pitch
        for x in range(cols):
            if line[4 * x + 2] == "-":
                rects.append((x * pitch - h, y - h, (x + 1) * pitch + h, y + h))
    for r in range(rows):
        line = case.maze[2 * r + 1]
        y0, y1 = (rows - r - 1) * pitch, (rows - r) * pitch
        for x in range(cols + 1):
            if line[4 * x] == "|":
                rects.append((x * pitch - h, y0 - h, x * pitch + h, y1 + h))
    segs = []
    for x0, y0, x1, y1 in rects:
        segs += [((x0, y0), (x1, y0)), ((x1, y0), (x1, y1)), ((x1, y1), (x0, y1)), ((x0, y1), (x0, y0))]
    return segs, pitch, rows, cols


# ---------------------------------------------------------------- simulacion


class MotionFailure(N.MotionFailureBase):
    """Movimiento incompleto: no continuar actualizando la posicion logica."""
