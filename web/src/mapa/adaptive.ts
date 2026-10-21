import type { Cell, Maze } from '../telemetria/types';
import { directions, shortestRoute, wall } from './maze';

export function toggleWall(m: Maze, x: number, y: number, d: number): Maze {
  const delta = directions[d];
  if (!delta || x < 0 || y < 0 || x >= m.W || y >= m.H) return m;
  const nx = x + delta[0], ny = y + delta[1];
  if (nx < 0 || ny < 0 || nx >= m.W || ny >= m.H) return m;
  const row = 2 * (m.H - 1 - y) + (d === 0 ? 0 : d === 2 ? 2 : 1);
  const col = 4 * x + (d === 3 ? 0 : d === 1 ? 4 : 1);
  const chars = [...m.map[row]];
  const closed = !wall(m, x, y, d);
  if (d === 0 || d === 2) for (let k = 0; k < 3; k++) chars[col + k] = closed ? '-' : ' ';
  else chars[col] = closed ? '|' : ' ';
  const map = [...m.map];
  map[row] = chars.join('');
  return { ...m, map };
}
export function plannedRoute(m: Maze, from: Cell): Cell[] {
  const route = shortestRoute(m, from);
  const end = route.at(-1);
  return end?.[0] === m.goal[0] && end[1] === m.goal[1] ? route : [];
}
