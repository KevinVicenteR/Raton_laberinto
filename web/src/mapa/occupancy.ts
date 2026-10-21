import type { RobotSample, SensorReadings } from '../telemetria/types';
import { currentReading, SENSOR_DEFS } from '../telemetria/sensorData';
export interface Occupancy {
  grid: Map<string, number>;
  last: RobotSample | null;
}
export const emptyOccupancy = (): Occupancy => ({ grid: new Map(), last: null });
export function updateOccupancy(
  previous: Occupancy,
  s: RobotSample,
  range = 600,
  readings: SensorReadings = {},
  geometry: Record<string, number> = {},
): Occupancy {
  if (
    !s.hasPose ||
    (previous.last &&
      Math.hypot(previous.last.x - s.x, previous.last.y - s.y) < 5 &&
      Math.abs(previous.last.a - s.a) < 4)
  )
    return previous;
  const grid = new Map(previous.grid),
    a = (s.a * Math.PI) / 180,
    fx = -Math.sin(a),
    fy = Math.cos(a),
    lx = -Math.cos(a),
    ly = -Math.sin(a),
    // Laterales montados a 45 grados; el VL53L0X ve un cono de 25: se marcan tres rayos del cono.
    angle = (45 * Math.PI) / 180,
    half = (12.5 * Math.PI) / 180;
  const bump = (x: number, y: number, delta: number) => {
    const key = `${Math.round(x / 10)},${Math.round(y / 10)}`;
    grid.set(key, Math.max(-2, Math.min(4, (grid.get(key) ?? 0) + delta)));
  };
  const ray = (x: number, y: number, dx: number, dy: number, dist: number) => {
    if (dist < 0) return;
    const hit = dist < range,
      len = Math.min(dist, range);
    let last = '';
    for (let t = 0; t < len - (hit ? 10 : 0); t += 7) {
      const key = `${Math.round((x + dx * t) / 10)},${Math.round((y + dy * t) / 10)}`;
      if (key !== last) bump(x + dx * t, y + dy * t, -0.4);
      last = key;
    }
    if (hit) bump(x + dx * len, y + dy * len, 1);
  };
  for (const sensor of SENSOR_DEFS) {
    const side = sensor.key === 'LI' || sensor.key === 'LD';
    const fallback = sensor.key === 'I' ? s.I : sensor.key === 'D' ? s.D : sensor.key === 'F' ? s.F : -1;
    const dist = currentReading(readings, sensor.key, s.t)?.mm ?? fallback;
    const sign = sensor.angle < 0 ? -1 : 1;
    const ahead =
      geometry[sensor.key === 'F' ? 'frontAheadMm' : side ? 'sideAheadMm' : 'diagAheadMm'] ?? sensor.ahead;
    const left =
      sensor.key === 'F'
        ? 0
        : sign * (geometry[side ? 'sideLateralMm' : 'diagLateralMm'] ?? Math.abs(sensor.left));
    const mount = side
      ? (sign * Math.PI) / 2
      : sensor.key === 'F'
        ? 0
        : sign * (geometry.diagMountDeg === undefined ? angle : (geometry.diagMountDeg * Math.PI) / 180);
    const fov = geometry.sensorFovDeg === undefined ? half : (geometry.sensorFovDeg * Math.PI) / 360;
    for (const da of [-fov, 0, fov]) {
      const b = mount + da;
      ray(
        s.x + fx * ahead + lx * left,
        s.y + fy * ahead + ly * left,
        fx * Math.cos(b) + lx * Math.sin(b),
        fy * Math.cos(b) + ly * Math.sin(b),
        dist,
      );
    }
  }
  // Keep memory bounded during long sessions.
  if (grid.size > 100000) for (const key of [...grid.keys()].slice(0, grid.size - 100000)) grid.delete(key);
  return { grid, last: s };
}
