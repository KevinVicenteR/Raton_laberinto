import { useEffect, useMemo, useState } from 'react';
import type { Cell, Maze } from '../telemetria/types';
import { competition } from './competition';
import { plannedRoute, toggleWall } from './adaptive';
import { routeDescription, wall } from './maze';

export function AdaptiveSolver() {
  const [maze, setMaze] = useState<Maze>(competition);
  const [position, setPosition] = useState<Cell>(competition.start);
  const [running, setRunning] = useState(false);
  const [edit, setEdit] = useState('walls');
  const route = useMemo(() => plannedRoute(maze, position), [maze, position]);
  const arrived = position[0] === maze.goal[0] && position[1] === maze.goal[1];
  useEffect(() => {
    if (!running || route.length < 2) return;
    const timer = setTimeout(() => setPosition(route[1]), 350);
    return () => clearTimeout(timer);
  }, [running, route]);
  useEffect(() => { if (route.length < 2) setRunning(false); }, [route]);
  const point = ([x, y]: Cell) => `${x + 0.5},${maze.H - y - 0.5}`;
  const edges = [];
  for (let y = 0; y < maze.H; y++) for (let x = 0; x < maze.W; x++) {
    for (const d of [0, 1, ...(y === 0 ? [2] : []), ...(x === 0 ? [3] : [])]) {
      const top = maze.H - y - 1;
      const x1 = x + (d === 1 ? 1 : 0), y1 = top + (d === 2 ? 1 : 0);
      const x2 = x1 + (d % 2 === 0 ? 1 : 0), y2 = y1 + (d % 2 === 1 ? 1 : 0);
      const border = (d === 0 && y === maze.H - 1) || (d === 1 && x === maze.W - 1) || d === 2 || d === 3;
      const closed = wall(maze, x, y, d);
      const change = () => setMaze(m => toggleWall(m, x, y, d));
      edges.push(<g key={`${x}-${y}-${d}`}>
        <line x1={x1} y1={y1} x2={x2} y2={y2} stroke={closed ? 'var(--text)' : 'var(--border)'} strokeWidth={closed ? 0.08 : 0.025} />
        {!border && edit === 'walls' && <line x1={x1} y1={y1} x2={x2} y2={y2} stroke="transparent" strokeWidth={0.28}
          role="button" tabIndex={0} aria-label={`${closed ? 'Abrir' : 'Cerrar'} pared ${x},${y} ${d === 0 ? 'norte' : 'este'}`}
          style={{ cursor: 'pointer' }} onClick={change} onKeyDown={e => { if (e.key === 'Enter' || e.key === ' ') { e.preventDefault(); change(); } }} />}
      </g>);
    }
  }
  return <section className="adaptive-solver">
    <h2>Resolver mapa con ruta adaptable</h2>
    <p>Calcula la ruta más corta del mapa de competencia. Al abrir o cerrar una pared, corrige el recorrido desde la posición actual.</p>
    <p className="status">Simulación por celdas en el navegador. No controla los motores ni reproduce la física o los sensores del robot.</p>
    <div className="solver-controls">
      <button disabled={route.length < 2} onClick={() => setRunning(v => !v)}>{running ? 'Pausar' : 'Resolver'}</button>
      <button disabled={running || route.length < 2} onClick={() => setPosition(route[1])}>Avanzar una celda</button>
      <button onClick={() => { setRunning(false); setPosition(maze.start); }}>Volver a la salida</button>
      <button onClick={() => { setRunning(false); setMaze(competition); setPosition(competition.start); }}>Restaurar mapa</button>
      <label>Editar: <select value={edit} onChange={e => setEdit(e.target.value)}>
        <option value="walls">Paredes</option><option value="goal">Meta</option><option value="position">Posición del ratón</option>
      </select></label>
    </div>
    <p role="status">{arrived ? 'Meta alcanzada.' : route.length ? `${route.length - 1} celdas hasta la meta.` : 'Sin camino: abre una pared o cambia la meta para continuar.'} Posición: ({position.join(', ')}).</p>
    <svg className="solver-map" viewBox={`-0.2 -0.2 ${maze.W + 0.4} ${maze.H + 0.4}`} aria-label="Mapa de competencia editable">
      {Array.from({ length: maze.W * maze.H }, (_, i) => {
        const x = i % maze.W, y = Math.floor(i / maze.W);
        const choose = () => { if (edit === 'walls') return; setRunning(false); if (edit === 'goal') setMaze(m => ({ ...m, goal: [x, y] })); else if (edit === 'position') setPosition([x, y]); };
        return <rect key={i} x={x} y={maze.H - y - 1} width={1} height={1} fill="var(--panel)"
          role={edit !== 'walls' ? 'button' : undefined} tabIndex={edit !== 'walls' ? 0 : undefined}
          aria-label={`Celda ${x},${y}`} onClick={choose} onKeyDown={e => { if (e.key === 'Enter' || e.key === ' ') { e.preventDefault(); choose(); } }} />;
      })}
      <polyline points={route.map(point).join(' ')} fill="none" stroke="var(--left)" strokeWidth={0.12} pointerEvents="none" />
      <circle cx={maze.goal[0] + 0.5} cy={maze.H - maze.goal[1] - 0.5} r={0.28} fill="var(--right)" pointerEvents="none" />
      <circle cx={position[0] + 0.5} cy={maze.H - position[1] - 0.5} r={0.22} fill="var(--front)" pointerEvents="none" />
      {edges}
    </svg>
    <p>Azul: ruta restante · Naranja: ratón · Verde: meta. En «Paredes», toca un borde interior para abrirlo o cerrarlo, incluso mientras avanza.</p>
    <p>{route.length > 1 ? routeDescription(route) : ''}</p>
  </section>;
}
