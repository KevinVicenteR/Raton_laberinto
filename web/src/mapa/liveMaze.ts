import type { Maze } from '../telemetria/types';

// CELL is in a start-relative frame centered on (20,20). NAV v2 is already in
// absolute maze millimeters. Convert only the wall cells, never the NAV pose.
export function liveMaze(
  params: Record<string, number>,
  cells: Record<string, number>,
  pitch?: number,
): Maze {
  const W = Math.max(1, Math.min(32, Math.round(params.mazeW ?? 19)));
  const H = Math.max(1, Math.min(32, Math.round(params.mazeH ?? 10)));
  const start: [number, number] = [params.startX ?? 18, params.startY ?? 0];
  const rows = Array.from({ length: H * 2 + 1 }, () => Array<string>(W * 4 + 1).fill(' '));
  for (const [key, mask] of Object.entries(cells)) {
    const [rx, ry] = key.split(',').map(Number);
    const x = rx - 20 + start[0],
      y = ry - 20 + start[1];
    if (x < 0 || y < 0 || x >= W || y >= H) continue;
    const row = 2 * (H - 1 - y);
    if (mask & 1) for (let k = 1; k < 4; k++) rows[row][x * 4 + k] = '-';
    if (mask & 4) for (let k = 1; k < 4; k++) rows[row + 2][x * 4 + k] = '-';
    if (mask & 2) rows[row + 1][x * 4 + 4] = '|';
    if (mask & 8) rows[row + 1][x * 4] = '|';
  }
  return {
    W,
    H,
    start,
    goal: [params.goalX ?? 4, params.goalY ?? 2],
    PITCH: pitch ?? params.cellMm ?? 155,
    WALL: params.wallMm ?? 12,
    map: rows.map((r) => r.join('')),
  };
}

