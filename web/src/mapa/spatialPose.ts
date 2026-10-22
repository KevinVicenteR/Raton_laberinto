import type { Cell, Maze, RobotSample } from '../telemetria/types';

// Live NAV is absolute, even when a transport frame has lost its optional P/H/G.
// Never switch origins or clamp an invalid estimate into a plausible corridor.
export function spatialPose(s: RobotSample | null, maze: Maze): Cell | null {
  if (!s?.hasPose || ![s.x, s.y, s.a, maze.PITCH].every(Number.isFinite) || maze.PITCH <= 0) return null;
  if (s.pitch !== undefined && (!Number.isFinite(s.pitch) || s.pitch < 100 || s.pitch > 500)) return null;
  const k = s.pitch === undefined ? 1 : maze.PITCH / s.pitch;
  const x = s.x * k,
    y = s.y * k;
  return x >= 0 && y >= 0 && x <= maze.W * maze.PITCH && y <= maze.H * maze.PITCH ? [x, y] : null;
}

export function spatialSegments(trail: RobotSample[], maze: Maze): [Cell, Cell][] {
  const result: [Cell, Cell][] = [];
  let previous: { p: Cell; t: number } | null = null;
  for (const s of trail) {
    const p = spatialPose(s, maze);
    if (!p) {
      previous = null;
      continue;
    }
    if (previous) {
      const dt = (s.t - previous.t) / 1000;
      // Pose corrections/reconnections are discontinuities, not traveled lines.
      if (dt > 0 && dt < 1.5 && Math.hypot(p[0] - previous.p[0], p[1] - previous.p[1]) <= 1200 * dt + 20)
        result.push([previous.p, p]);
    }
    previous = { p, t: s.t };
  }
  return result;
}
