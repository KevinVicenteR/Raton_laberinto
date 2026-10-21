import { describe, it, expect } from 'vitest';
import { renderToStaticMarkup } from 'react-dom/server';
import { parse } from '../../conexion/protocol';
import { initialState, telemetryReducer } from '../state';
import { LiveMap2D } from '../../mapa/LiveMap';
import { liveMaze } from '../../mapa/liveMaze';
import { wall } from '../../mapa/maze';

describe('Autonomía y resincronización', () => {
  it('interpreta incertidumbre espacial, salud y decisiones', () => {
    expect(
      parse('HEALTH phase=EN_MARCHA reason=VIGILANDO age=1,2,3,4,5 sigma=3,4 heading=0 repairs=1')?.age,
    ).toEqual([1, 2, 3, 4, 5]);
    expect(
      parse('POSE x=100 y=200 sigma=3,4 visited=1 confirmed=0 collisions=2 oscillations=1 phase=ESTIMADA')
        ?.sigma,
    ).toEqual([3, 4]);
    expect(parse('POSE x=NaN')).toBeNull();
    const state = telemetryReducer(initialState(), {
      type: 'line',
      line: 'AUTO phase=REPARACION reason=REINICIAR_XSHUT sensor=1',
      t: 0,
    });
    expect(state.decision).toContain('REPARACION');
    expect(state.events).toHaveLength(1);
  });
  it('recupera el mapa sin sobrescribir una celda nueva con una instantánea anterior', () => {
    let state = initialState();
    for (const line of [
      'SYNC_BEGIN',
      'FIRMWARE autonomia-v3',
      'CELL x=20 y=20 w=3',
      'SYNC_CELL x=20 y=20 w=1',
      'SYNC_CELL x=21 y=20 w=4',
      'SYNC_DONE overwritten=0',
    ])
      state = telemetryReducer(state, { type: 'line', line, t: 100 });
    expect(state.syncing).toBe(false);
    expect(state.firmwareV3).toBe(true);
    expect(state.cells).toEqual({ '20,20': 3, '21,20': 4 });
  });
  it('no duplica eventos históricos al reconectar dos veces', () => {
    let state = initialState();
    const line = 'HISTORY id=7 ms=900 AUTO phase=RECUPERADO reason=ATASCO';
    state = telemetryReducer(state, { type: 'line', line, t: 100 });
    state = telemetryReducer(state, { type: 'line', line, t: 200 });
    expect(state.events).toHaveLength(1);
  });
  it('sitúa las paredes aprendidas y conserva las coordenadas NAV absolutas', () => {
    const m = liveMaze({ mazeW: 3, mazeH: 3, startX: 2, startY: 0 }, { '20,20': 9 }, 160);
    expect(wall(m, 2, 0, 0)).toBe(true);
    expect(wall(m, 2, 0, 3)).toBe(true);
    expect(wall(m, 0, 0, 0)).toBe(false); // unknown is drawn empty, never marked confirmed
    const html = renderToStaticMarkup(<LiveMap2D maze={m} cells={{ '20,20': 9 }} trail={[]} last={null} />);
    expect(html).toContain('Mapa aprendido 2D');
    expect(html).toContain('480');
  });
});
