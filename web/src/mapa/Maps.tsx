import { useState } from 'react';
import { wall, shortestRoute, floodFill, worldPose, poseCell, routeDescription } from './maze';
import type { Cell, Maze, RobotSample, ReplayData } from '../telemetria/types';
const points = (cells: Cell[], h: number) =>
  cells.map(([x, y]) => `${(x + 0.5) * 20},${(h - y - 0.5) * 20}`).join(' ');
export function Track({
  maze,
  last,
  trail,
  onGoal,
  replaying = false,
}: {
  maze: Maze;
  last: RobotSample | null;
  trail: RobotSample[];
  onGoal: (c: Cell) => void;
  replaying?: boolean;
}) {
  const [offset, setOffset] = useState<Cell>([0, 0]),
    [action, setAction] = useState('pose');
  const pose = last ? worldPose(last, maze, offset) : null,
    cell = pose && last ? poseCell(pose, last.a, maze.PITCH, !!last.pitch) : maze.start,
    dist = floodFill(maze)[cell[0]]?.[cell[1]],
    route = shortestRoute(maze, maze.start),
    remaining = shortestRoute(maze, cell);
  const path = trail
    .map((s) => worldPose(s, maze, offset))
    .map(([x, y]) => `${(x / maze.PITCH) * 20},${(maze.H - y / maze.PITCH) * 20}`)
    .join(' ');
  return (
    <section className="panel wide">
      <h2>Pista de competencia</h2>
      <div className="row">
        <span>
          Celda: {cell.join(', ')} · Hasta la meta:{' '}
          {Number.isFinite(dist) ? dist : 'Fuera de pista / sin ruta'} · Ruta mínima: {route.length - 1}{' '}
          celdas
        </span>
        <select
          aria-label="Acción al tocar la pista"
          value={action}
          disabled={replaying}
          onChange={(e) => setAction(e.target.value)}
        >
          <option value="pose">Clic: recolocar vista del ratón</option>
          <option value="goal">Clic: fijar meta</option>
        </select>
        <button disabled={replaying} onClick={() => setOffset([0, 0])}>
          Restaurar posición
        </button>
      </div>
      <svg
        className="maze"
        role="img"
        aria-label="Pista de competencia interactiva"
        viewBox={`-2 -2 ${maze.W * 20 + 4} ${maze.H * 20 + 4}`}
      >
        {Array.from({ length: maze.W * maze.H }, (_, i) => {
          const x = i % maze.W,
            y = Math.floor(i / maze.W),
            top = (maze.H - y - 1) * 20;
          return (
            <g key={i}>
              <rect
                className={
                  x === maze.goal[0] && y === maze.goal[1]
                    ? 'goal'
                    : x === maze.start[0] && y === maze.start[1]
                      ? 'start'
                      : 'cell'
                }
                x={x * 20}
                y={top}
                width="20"
                height="20"
                onClick={() => {
                  if (replaying) return;
                  if (action === 'goal') onGoal([x, y]);
                  else if (pose && last) {
                    const a = (last.a * Math.PI) / 180;
                    setOffset([
                      offset[0] + (x + 0.5) * maze.PITCH + Math.sin(a) * 33 - pose[0],
                      offset[1] + (y + 0.5) * maze.PITCH - Math.cos(a) * 33 - pose[1],
                    ]);
                  }
                }}
              />
              {[0, 1, 2, 3].map((d) =>
                wall(maze, x, y, d) ? (
                  <line
                    key={d}
                    className="wall"
                    x1={x * 20 + (d === 1 ? 20 : 0)}
                    y1={top + (d === 2 ? 20 : 0)}
                    x2={x * 20 + (d === 3 ? 0 : 20)}
                    y2={top + (d === 0 ? 0 : 20)}
                  />
                ) : null,
              )}
            </g>
          );
        })}
        <polyline className="route" points={points(route, maze.H)} />
        <polyline className="remaining" points={points(remaining, maze.H)} />
        <polyline className="trail" points={path} />
        {pose && (
          <g
            transform={`translate(${(pose[0] / maze.PITCH) * 20} ${(maze.H - pose[1] / maze.PITCH) * 20}) rotate(${-last!.a})`}
          >
            <path d="M0,-7 L-4,5 L4,5 Z" fill="var(--front)" />
          </g>
        )}
      </svg>
      <p>Ruta mínima: {routeDescription(route)}</p>
      <p className="status">
        Verde: salida · violeta: meta · amarillo: ruta mínima · rosa: ruta restante · azul: recorrido. La
        recolocación ajusta solo la vista.
      </p>
    </section>
  );
}
export function LearnedMap({
  cells,
  robot,
  goal,
  onGoal,
  disabled,
}: {
  cells: Record<string, number>;
  robot: Cell;
  goal: Cell | null;
  onGoal: (cell: Cell) => void;
  disabled: boolean;
}) {
  const keys = Object.keys(cells)
      .map((k) => k.split(',').map(Number))
      .concat([[20, 20], robot], goal ? [goal] : []),
    xs = keys.map((c) => c[0]),
    ys = keys.map((c) => c[1]),
    x0 = Math.max(0, Math.min(...xs) - 2),
    y0 = Math.max(0, Math.min(...ys) - 2),
    x1 = Math.min(39, Math.max(...xs) + 2),
    y1 = Math.min(39, Math.max(...ys) + 2);
  return (
    <>
      <p>
        {Object.keys(cells).length} celdas mapeadas · Meta: {goal?.join(', ') ?? 'toca una celda'}
      </p>
      <svg
        className="maze learned"
        role="img"
        aria-label="Mapa aprendido; toca una celda para elegir meta"
        viewBox={`-2 -2 ${(x1 - x0 + 1) * 20 + 4} ${(y1 - y0 + 1) * 20 + 4}`}
      >
        {Array.from({ length: (x1 - x0 + 1) * (y1 - y0 + 1) }, (_, i) => {
          const x = x0 + (i % (x1 - x0 + 1)),
            y = y0 + Math.floor(i / (x1 - x0 + 1)),
            px = (x - x0) * 20,
            py = (y1 - y) * 20,
            w = cells[`${x},${y}`];
          return (
            <g key={i}>
              <rect
                x={px}
                y={py}
                width="20"
                height="20"
                className={
                  goal?.[0] === x && goal[1] === y
                    ? 'goal'
                    : x === 20 && y === 20
                      ? 'start'
                      : w & 16
                        ? 'dead'
                        : w !== undefined
                          ? 'seen'
                          : 'cell'
                }
                onClick={() => !disabled && onGoal([x, y])}
              />
              {[1, 2, 4, 8].map((bit, d) =>
                w & bit ? (
                  <line
                    key={bit}
                    className="wall"
                    x1={px + (d === 1 ? 20 : 0)}
                    y1={py + (d === 2 ? 20 : 0)}
                    x2={px + (d === 3 ? 0 : 20)}
                    y2={py + (d === 0 ? 0 : 20)}
                  />
                ) : null,
              )}
            </g>
          );
        })}
        <circle cx={(robot[0] - x0 + 0.5) * 20} cy={(y1 - robot[1] + 0.5) * 20} r="5" fill="var(--left)" />
      </svg>
    </>
  );
}
export function SimulationMap({ data, time }: { data: ReplayData; time: number }) {
  const width = data.width ?? 3000,
    height = data.height ?? 1600,
    frames = data.trace.filter((f) => f.t <= time),
    sample = frames.at(-1) ?? data.trace[0];
  return (
    <svg
      className="maze"
      viewBox={`-20 -20 ${width + 40} ${height + 40}`}
      role="img"
      aria-label="Recorrido de la simulación"
    >
      {data.walls?.map(([a, b], i) => (
        <line
          key={i}
          x1={a[0]}
          y1={height - a[1]}
          x2={b[0]}
          y2={height - b[1]}
          stroke="var(--text)"
          strokeWidth="3"
        />
      ))}
      {data.goal && (
        <rect
          x={data.goal[0] * (data.pitch ?? 155)}
          y={height - (data.goal[1] + 1) * (data.pitch ?? 155)}
          width={data.pitch ?? 155}
          height={data.pitch ?? 155}
          className="goal"
        />
      )}
      <polyline
        points={frames
          .filter((f) => f.truth)
          .map((f) => `${f.truth![0]},${height - f.truth![1]}`)
          .join(' ')}
        fill="none"
        stroke="var(--left)"
        strokeWidth="8"
      />
      {data.events
        ?.filter((e) => e[0] <= time && e[1] === 'choque')
        .map((e, i) => (
          <text key={i} x={e[2]} y={height - e[3]} fill="var(--danger)" fontSize="60">
            ×
          </text>
        ))}
      {sample.truth && (
        <g
          transform={`translate(${sample.truth[0]} ${height - sample.truth[1]}) rotate(${(-sample.truth[2] * 180) / Math.PI})`}
        >
          <rect x="-17" y="-46" width="100" height="92" fill="var(--front)" />
          <circle cx="65" r="8" fill="white" />
        </g>
      )}
    </svg>
  );
}
