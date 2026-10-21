import { describe, expect, it } from 'vitest';
import { competition } from '../competition';
import { plannedRoute } from '../adaptive';
import { wall } from '../maze';
import source from '../../../../mazes/reglamento_2026.txt?raw';

describe('mapa de referencia compartido', () => {
  it('usa el mismo trazado que el simulador y conserva dimensiones y bordes', () => {
    expect(competition.map).toEqual(source.split(/\r?\n/).filter((l) => l && !l.startsWith('#')));
    expect(competition.map).toHaveLength(competition.H * 2 + 1);
    expect(competition.map.every((l) => l.length === competition.W * 4 + 1)).toBe(true);
    expect(competition.WALL).toBe(12);
    for (let x = 0; x < competition.W; x++) {
      expect(wall(competition, x, 0, 2)).toBe(true);
      expect(wall(competition, x, competition.H - 1, 0)).toBe(true);
    }
    expect(plannedRoute(competition, competition.start).at(-1)).toEqual(competition.goal);
  });
});

import { trackFrame3D, mazeFromReplay } from '../maze';
it('alinea el mapa 3D con NAV absoluto y mantiene el origen de registros antiguos', () => {
  const absolute = trackFrame3D(competition, 161.5);
  expect(absolute.origin).toEqual([0, 0]);
  expect((competition.start[0] + 0.5) * competition.PITCH * absolute.scale).toBeCloseTo(18.5 * 161.5);
  const old = trackFrame3D(competition);
  expect(old.scale).toBe(1);
  expect(old.origin).toEqual([18.5 * 155, 0.5 * 155 - 33]);
  const replay = mazeFromReplay(
    { maze: competition.map, start: competition.start, goal: competition.goal, pitch: 152, corridor_mm: 140 },
    competition,
  );
  expect(replay.WALL).toBe(12);
});
