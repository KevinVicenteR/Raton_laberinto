import type { RobotSample } from '../telemetria/types';
export interface Occupancy {
  grid: Map<string, number>;
  last: RobotSample | null;
}
export const emptyOccupancy = (): Occupancy => ({ grid: new Map(), last: null });
export function updateOccupancy(previous: Occupancy, s: RobotSample, range = 300): Occupancy {
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
  const [L, F, R] = s.raw ?? [s.I, s.F, s.D];
  for (const da of [-half, 0, half]) {
    ray(
      s.x + fx * 40,
      s.y + fy * 40,
      fx * Math.cos(da) + lx * Math.sin(da),
      fy * Math.cos(da) + ly * Math.sin(da),
      F,
    );
    for (const [sign, v] of [
      [1, L],
      [-1, R],
    ]) {
      const b = angle * sign + da;
      ray(
        s.x + fx * 30 + lx * 30 * sign,
        s.y + fy * 30 + ly * 30 * sign,
        fx * Math.cos(b) + lx * Math.sin(b),
        fy * Math.cos(b) + ly * Math.sin(b),
        v,
      );
    }
  }
  // Keep memory bounded during long sessions.
  if (grid.size > 100000) for (const key of [...grid.keys()].slice(0, grid.size - 100000)) grid.delete(key);
  return { grid, last: s };
}
