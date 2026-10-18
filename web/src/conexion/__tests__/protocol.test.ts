import { describe, it, expect } from 'vitest';
import * as p from '../protocol';
describe('Protocolo y sesiones: aserciones de la página original', () => {
  it('interpreta observaciones y maniobras', () => {
    expect(p.parse('1.20\tOBS raw=-1,50,60 age=100,0,20 phase=GIRO drops=4')?.raw).toEqual([-1, 50, 60]);
    expect(p.parse('MOTION completed=0 reason=2 actual=14')?.completed).toBe(0);
    expect(p.parse('SWEEP observed=1 feasible=0 entry=-5')).toBeNull();
    expect(p.parse('garbage')).toBeNull();
    expect(p.parse('OBS raw=1,NaN,3')).toBeNull();
    expect(p.parse('OBS age=NaN')).toBeNull();
    expect(p.parse('OBS raw=1,2')).toBeNull();
  });
  const sample = { t: 0, filtered: { L: -1, F: 20, R: 50 }, truth: [1, 2, 0] };
  it('valida registros y rechaza todas las contradicciones originales', () => {
    expect(p.validateReplay({ trace: [sample] }).trace.length).toBe(1);
    expect(() => p.validateReplay({ trace: [] })).toThrow();
    expect(() => p.validateReplay({ trace: [{ ...sample, t: 1 }, sample] })).toThrow();
    expect(() => p.validateReplay({ trace: [{ ...sample, truth: [NaN, 0, 0] }] })).toThrow();
    expect(() =>
      p.validateReplay({ trace: [sample], maze: ['+---+', '|   |', '+---+'], goal: [4, 2] }),
    ).toThrow();
    expect(() =>
      p.validateReplay({ trace: [sample], clean_arrival: true, reached: null, events: [] }),
    ).toThrow();
    expect(() =>
      p.validateReplay({ trace: [sample], clean_arrival: true, reached: 1, events: [[0.5, 'choque', 1, 2]] }),
    ).toThrow();
  });
  it('CRC y tramas idénticas al firmware', () => {
    expect(p.crc8('123456789')).toBe(0xf4);
    const fr = p.frame(7, 'm3');
    expect(fr).toBe('$7,m3*' + p.crc8('7,m3').toString(16).toUpperCase().padStart(2, '0'));
    expect(p.unframe(fr)).toEqual({ kind: 'ok', seq: 7, payload: 'm3' });
    expect(p.unframe(fr.replace('m3', 'm4')).kind).toBe('bad');
    expect(p.unframe('$7,m3*G1').kind).toBe('bad');
    expect(p.unframe('INICIO_MAPA')).toEqual({ kind: 'plain', payload: 'INICIO_MAPA' });
    expect(p.unframe(p.frame(65535, 'x'))).toMatchObject({ seq: 65535 });
  });
});
