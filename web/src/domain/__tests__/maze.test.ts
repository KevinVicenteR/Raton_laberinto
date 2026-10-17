import { describe, it, expect } from 'vitest';
import { competition } from '../competition';
import { floodFill, shortestRoute, wall, directions } from '../maze';
import type { Maze } from '../types';
describe('Flood fill', () => {
  const maze: Maze = {
    W: 2,
    H: 2,
    PITCH: 155,
    start: [0, 0],
    goal: [1, 1],
    map: ['+---+---+', '|       |', '+   +   +', '|       |', '+---+---+'],
  };
  it('calcula distancias mínimas y la ruta sin atravesar paredes', () => {
    expect(floodFill(maze)).toEqual([
      [2, 1],
      [1, 0],
    ]);
    expect(shortestRoute(maze, [0, 0])).toEqual([
      [0, 0],
      [0, 1],
      [1, 1],
    ]);
  });
  it('deja infinito en zonas cerradas y protege metas fuera de mapa', () => {
    const closed = { ...maze, map: ['+---+---+', '|   |   |', '+---+---+', '|   |   |', '+---+---+'] };
    expect(floodFill(closed)[0][0]).toBe(Infinity);
    expect(shortestRoute(closed, [0, 0])).toEqual([[0, 0]]);
    expect(floodFill(maze, [9, 9])[0][0]).toBe(Infinity);
  });
  it('la ruta de competencia alcanza la meta y cada paso baja exactamente una unidad', () => {
    const route = shortestRoute(competition, competition.start),
      dist = floodFill(competition);
    expect(route.at(-1)).toEqual(competition.goal);
    expect(route.length - 1).toBe(dist[18][0]);
    for (let i = 1; i < route.length; i++) {
      const [x, y] = route[i - 1],
        [nx, ny] = route[i],
        d = directions.findIndex(([dx, dy]) => x + dx === nx && y + dy === ny);
      expect(d).toBeGreaterThanOrEqual(0);
      expect(wall(competition, x, y, d)).toBe(false);
      expect(dist[nx][ny]).toBe(dist[x][y] - 1);
    }
  });
});
