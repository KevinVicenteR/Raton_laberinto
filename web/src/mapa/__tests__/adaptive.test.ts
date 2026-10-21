import { describe, expect, it } from 'vitest';
import type { Maze } from '../../telemetria/types';
import { competition } from '../competition';
import { plannedRoute, toggleWall } from '../adaptive';
import { wall } from '../maze';
const square: Maze = { W: 2, H: 2, PITCH: 155, start: [0, 0], goal: [1, 1], map: ['+---+---+', '|       |', '+   +   +', '|       |', '+---+---+'] };
describe('ruta adaptable', () => {
  it('resuelve competencia sin atravesar paredes', () => {
    const route = plannedRoute(competition, competition.start);
    expect(route.length).toBeGreaterThan(1);
    expect(route.at(-1)).toEqual(competition.goal);
    for (let i = 1; i < route.length; i++) {
      const [x, y] = route[i - 1], [nx, ny] = route[i];
      expect(Math.abs(nx - x) + Math.abs(ny - y)).toBe(1);
      expect(wall(competition, x, y, ny > y ? 0 : nx > x ? 1 : ny < y ? 2 : 3)).toBe(false);
    }
  });
  it('cambia de direccion al cerrar el siguiente paso', () => {
    expect(plannedRoute(square, [0, 0])[1]).toEqual([0, 1]);
    const changed = toggleWall(square, 0, 0, 0);
    expect(wall(changed, 0, 1, 2)).toBe(true);
    expect(plannedRoute(changed, [0, 0])).toEqual([[0, 0], [1, 0], [1, 1]]);
    expect(wall(square, 0, 0, 0)).toBe(false);
    expect(toggleWall(changed, 0, 0, 0)).toEqual(square);
  });
  it('puede retroceder desde la posicion actual y se detiene sin salida', () => {
    const changed = toggleWall(square, 0, 1, 1);
    expect(plannedRoute(changed, [0, 1])).toEqual([[0, 1], [0, 0], [1, 0], [1, 1]]);
    expect(plannedRoute(toggleWall(changed, 0, 1, 2), [0, 1])).toEqual([]);
  });
  it('conserva los bordes y reconoce la llegada', () => {
    expect(toggleWall(square, 0, 0, 3)).toBe(square);
    expect(plannedRoute(square, square.goal)).toEqual([square.goal]);
  });
});
