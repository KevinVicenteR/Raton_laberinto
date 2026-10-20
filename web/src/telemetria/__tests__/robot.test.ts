import { describe, expect, it } from 'vitest';
import { parseRobot } from '../robot';
import { poseCell, worldPose } from '../../mapa/maze';
import type { Maze } from '../types';

describe('parseRobot', () => {
  it('lee la línea NAV del firmware v2 con pose absoluta', () => {
    const s = parseRobot('NAV I52 F-1 D60 v398 r12 p143 x2867 y1472 a90 P1550 H1800 G1', 0);
    expect(s).not.toBeNull();
    expect(s!.I).toBe(52);
    expect(s!.F).toBe(-1);
    expect(s!.x).toBe(2867);
    expect(s!.a).toBe(90);
    expect(s!.pitch).toBe(155);
    expect(s!.hasPose).toBe(true);
  });
  it('sigue leyendo registros antiguos sin P/H/G', () => {
    const s = parseRobot('NAV I52 F100 D60 v300 r0 p143 x10 y20 a0', 0);
    expect(s?.pitch).toBeUndefined();
    expect(s?.hasPose).toBe(true);
  });
  it('coloca la pose v2 en la celda correcta', () => {
    const s = parseRobot('NAV I52 F-1 D60 v398 r12 p143 x2867 y1472 a90 P1550 H1800 G1', 0)!;
    const maze = { PITCH: 155 } as unknown as Maze;
    const p = worldPose(s, maze);
    expect(poseCell(p, s.a, maze.PITCH, true)).toEqual([18, 9]);
  });
});
