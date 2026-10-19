import { describe, it, expect } from 'vitest';
import { initialSupervisor, supervise } from '../supervisor';
import { emptyOccupancy, updateOccupancy } from '../../mapa/occupancy';
import type { RobotSample } from '../../telemetria/types';
const sample = (t: number, state = 'RECTA'): RobotSample => ({
  t,
  state,
  I: 60,
  F: 100,
  D: 60,
  v: 100,
  r: 0,
  p: 140,
  x: 0,
  y: 0,
  a: 0,
  hasPose: true,
});
describe('Supervisor y mapa de ocupación', () => {
  it('aprende el umbral lateral sin intervención del supervisor', () => {
    let state = initialSupervisor();
    const commands: string[] = [];
    for (let i = 0; i < 65; i++) {
      const result = supervise(state, sample(i * 100), 3, false, true, 100);
      state = result.state;
      commands.push(...result.commands);
    }
    expect(commands).toEqual(['w41']);
    expect(state.wall).toBe(41);
  });
  it('advierte de giros en los modos por celdas sin cambiar modo', () => {
    let state = initialSupervisor();
    const commands: string[] = [];
    for (let i = 0; i < 30; i++) {
      const result = supervise(state, sample(5000 + i * 100, i % 2 ? 'GIRO' : 'RECTA'), 3, true, false, 100);
      state = result.state;
      commands.push(...result.commands);
    }
    expect(commands).not.toContain('m1');
    expect(state.message).toContain('solo aviso');
  });
  it('en otros modos sale del bucle con parada y pared derecha', () => {
    let state = initialSupervisor();
    const commands: string[] = [];
    for (let i = 0; i < 30; i++) {
      const result = supervise(state, sample(5000 + i * 100, i % 2 ? 'GIRO' : 'RECTA'), 5, true, false, 100);
      state = result.state;
      commands.push(...result.commands);
    }
    expect(commands).toEqual(['x', 'm1', 's1']);
  });
  it('sin lectura no se demuestra ni pared ni espacio libre', () => {
    const s = { ...sample(0), I: -1, F: -1, D: -1 };
    const next = updateOccupancy(emptyOccupancy(), s);
    expect(next.grid.size).toBe(0);
  });
  it('no acumula repetidamente la misma observación estando parado', () => {
    const a = updateOccupancy(emptyOccupancy(), sample(0));
    expect(a.grid.size).toBeGreaterThan(0);
    expect(updateOccupancy(a, sample(100))).toBe(a);
  });
});
