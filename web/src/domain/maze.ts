import type { Cell, Maze, RobotSample } from './types';
export const directions = [
  [0, 1],
  [1, 0],
  [0, -1],
  [-1, 0],
] as const;
export function wall(m: Maze, x: number, y: number, d: number): boolean {
  const r = m.H - 1 - y;
  if (x < 0 || y < 0 || x >= m.W || y >= m.H) return true;
  return d === 0
    ? m.map[2 * r][4 * x + 2] === '-'
    : d === 2
      ? m.map[2 * r + 2][4 * x + 2] === '-'
      : d === 3
        ? m.map[2 * r + 1][4 * x] === '|'
        : m.map[2 * r + 1][4 * x + 4] === '|';
}
export function floodFill(m: Maze, goal: Cell = m.goal): number[][] {
  const dist = Array.from({ length: m.W }, () => Array<number>(m.H).fill(Infinity));
  if (!dist[goal[0]] || goal[1] < 0 || goal[1] >= m.H) return dist;
  const q: Cell[] = [goal];
  dist[goal[0]][goal[1]] = 0;
  for (let i = 0; i < q.length; i++) {
    const [x, y] = q[i];
    directions.forEach(([dx, dy], d) => {
      const nx = x + dx,
        ny = y + dy;
      if (!wall(m, x, y, d) && nx >= 0 && ny >= 0 && nx < m.W && ny < m.H && dist[nx][ny] === Infinity) {
        dist[nx][ny] = dist[x][y] + 1;
        q.push([nx, ny]);
      }
    });
  }
  return dist;
}
export function shortestRoute(m: Maze, from: Cell, goal = m.goal): Cell[] {
  const dist = floodFill(m, goal),
    route: Cell[] = [from];
  let [x, y] = from;
  if (!Number.isFinite(dist[x]?.[y])) return route;
  while (dist[x][y] > 0) {
    const d = directions.findIndex(
      ([dx, dy], d) => !wall(m, x, y, d) && dist[x + dx]?.[y + dy] === dist[x][y] - 1,
    );
    if (d < 0) break;
    x += directions[d][0];
    y += directions[d][1];
    route.push([x, y]);
  }
  return route;
}
export function worldPose(s: RobotSample, m: Maze, offset: Cell = [0, 0]): Cell {
  return [
    s.x + (m.start[0] + 0.5) * m.PITCH + offset[0],
    s.y + (m.start[1] + 0.5) * m.PITCH - 33 + offset[1],
  ];
}
export function poseCell(p: Cell, a: number, pitch: number): Cell {
  const r = (a * Math.PI) / 180;
  return [Math.floor((p[0] - Math.sin(r) * 33) / pitch), Math.floor((p[1] + Math.cos(r) * 33) / pitch)];
}
export function mazeFromReplay(
  data: {
    maze?: string[];
    start?: Cell;
    goal?: Cell;
    pitch?: number;
  },
  fallback: Maze,
): Maze {
  return data.maze
    ? {
        map: data.maze,
        W: (data.maze[0].length - 1) / 4,
        H: (data.maze.length - 1) / 2,
        start: data.start!,
        goal: data.goal!,
        PITCH: data.pitch ?? 155,
      }
    : fallback;
}

export function routeDescription(route: Cell[]): string {
  const names: Record<string, string> = { '0,1': 'norte', '1,0': 'este', '0,-1': 'sur', '-1,0': 'oeste' };
  const legs: { direction: string; count: number }[] = [];
  for (let i = 1; i < route.length; i++) {
    const direction = `${route[i][0] - route[i - 1][0]},${route[i][1] - route[i - 1][1]}`;
    const previous = legs.at(-1);
    if (previous?.direction === direction) previous.count++;
    else legs.push({ direction, count: 1 });
  }
  return legs.map((leg) => `${leg.count} al ${names[leg.direction]}`).join(' → ') || 'En la meta / sin ruta';
}
