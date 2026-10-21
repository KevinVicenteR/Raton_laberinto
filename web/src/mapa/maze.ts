import type { Cell, Maze, RobotSample } from '../telemetria/types';
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
  // v2: eje del raton en el laberinto, escalado al paso de esta pista.
  if (s.pitch) {
    const k = m.PITCH / s.pitch;
    return [s.x * k + offset[0], s.y * k + offset[1]];
  }
  return [
    s.x + (m.start[0] + 0.5) * m.PITCH + offset[0],
    s.y + (m.start[1] + 0.5) * m.PITCH - 33 + offset[1],
  ];
}
export function poseCell(p: Cell, a: number, pitch: number, absolute = false): Cell {
  if (absolute) return [Math.floor(p[0] / pitch), Math.floor(p[1] / pitch)];
  const r = (a * Math.PI) / 180;
  return [Math.floor((p[0] - Math.sin(r) * 33) / pitch), Math.floor((p[1] + Math.cos(r) * 33) / pitch)];
}
export function mazeFromReplay(
  data: {
    maze?: string[];
    start?: Cell;
    goal?: Cell;
    pitch?: number;
    corridor_mm?: number;
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
        WALL:
          typeof data.corridor_mm === 'number' &&
          data.corridor_mm >= 0 &&
          data.corridor_mm < (data.pitch ?? 155)
            ? (data.pitch ?? 155) - data.corridor_mm
            : (fallback.WALL ?? 12),
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

// NAV v2 da coordenadas absolutas en unidades de su paso aprendido; los registros
// antiguos daban coordenadas relativas a la salida. El mapa 3D debe usar el mismo marco.
export function trackFrame3D(maze: Maze, pitch?: number) {
  return pitch && pitch > 0
    ? { scale: pitch / maze.PITCH, origin: [0, 0] as Cell }
    : {
        scale: 1,
        origin: [(maze.start[0] + 0.5) * maze.PITCH, (maze.start[1] + 0.5) * maze.PITCH - 33] as Cell,
      };
}
