import type { Maze, RobotSample } from '../telemetria/types';
import { wall } from './maze';
import { spatialPose, spatialSegments } from './spatialPose';

export function LiveMap2D({
  maze,
  cells,
  trail,
  last,
}: {
  maze: Maze;
  cells: Record<string, number>;
  trail: RobotSample[];
  last: RobotSample | null;
}) {
  const p = maze.PITCH;
  const pose = spatialPose(last, maze);
  return (
    <svg
      className="maze"
      viewBox={`-10 -10 ${maze.W * p + 20} ${maze.H * p + 20}`}
      role="img"
      aria-label="Mapa aprendido 2D y trayectoria estimada"
    >
      {Array.from({ length: maze.W * maze.H }, (_, i) => {
        const x = i % maze.W,
          y = Math.floor(i / maze.W);
        const observed = cells[`${x - maze.start[0] + 20},${y - maze.start[1] + 20}`] !== undefined;
        return (
          <g key={i}>
            <rect
              x={x * p}
              y={(maze.H - y - 1) * p}
              width={p}
              height={p}
              fill={observed ? 'var(--panel)' : 'var(--grid)'}
              stroke="var(--border)"
            />
            {[0, 1, 2, 3].map((d) =>
              wall(maze, x, y, d) ? (
                <line
                  key={d}
                  stroke="var(--text)"
                  strokeWidth={maze.WALL ?? 12}
                  x1={(x + (d === 1 ? 1 : 0)) * p}
                  y1={(maze.H - y - (d === 2 ? 0 : 1)) * p}
                  x2={(x + (d === 3 ? 0 : 1)) * p}
                  y2={(maze.H - y - (d === 0 ? 1 : 0)) * p}
                />
              ) : null,
            )}
          </g>
        );
      })}
      <path
        d={spatialSegments(trail, maze)
          .map(([a, b]) => `M${a[0]},${maze.H * p - a[1]} L${b[0]},${maze.H * p - b[1]}`)
          .join(' ')}
        fill="none"
        stroke="var(--left)"
        strokeWidth="5"
      />
      {last && pose && (
        <g transform={`translate(${pose[0]},${maze.H * p - pose[1]}) rotate(${-last.a})`}>
          <circle r="24" fill="var(--front)" />
          <path d="M0,-32 L-10,-12 L10,-12 Z" fill="var(--text)" />
        </g>
      )}
    </svg>
  );
}
