import { describe, it, expect } from 'vitest';
import { resetRobot } from '../resetRobot';
import { initialState, telemetryReducer } from '../../telemetria/state';
const line = (s: ReturnType<typeof initialState>, text: string) =>
  telemetryReducer(s, { type: 'line', line: text, t: 10 });
describe('Reinicio completo confirmado por la placa', () => {
  it('cancela órdenes pendientes y detiene antes de solicitar el borrado', async () => {
    const calls: string[] = [];
    expect(
      await resetRobot(
        async (cmd) => {
          calls.push(cmd);
          return true;
        },
        () => {
          calls.push('cancel');
        },
      ),
    ).toBe(true);
    expect(calls).toEqual(['cancel', 'x', 'resetall']);
    calls.length = 0;
    expect(
      await resetRobot(
        async (cmd) => {
          calls.push(cmd);
          return false;
        },
        () => {},
      ),
    ).toBe(false);
    expect(calls).toEqual(['x']);
  });
  it('conserva el mapa hasta confirmar el borrado y descarta tramas antiguas después', () => {
    let s = line(initialState(), 'CELL x=20 y=20 w=3');
    s = { ...s, boardParams: { cellMm: 200 }, raceStart: 20, record: ['old'], historyLines: ['old'] };
    s = line(s, 'REINICIO_TOTAL_SOLICITADO');
    expect(s.resetPending).toBe(true);
    expect(s.cells['20,20']).toBe(3);
    s = line(s, 'REINICIO_TOTAL_OK');
    expect(s.cells).toEqual({});
    expect(s.boardParams).toEqual({});
    expect(s.trail).toEqual([]);
    expect(s.record).toEqual([]);
    expect(s.historyLines).toEqual([]);
    expect(s.raceStart).toBeNull();
    expect(s.resetRevision).toBe(1);
    expect(s.awaitingResetBoot).toBe(true);
    expect(line(s, 'CELL x=20 y=20 w=3')).toBe(s);
    s = line(s, 'FIRMWARE autonomia-v3');
    expect(s.awaitingResetBoot).toBe(false);
    expect(line(s, 'CELL x=20 y=20 w=0').cells).toEqual({ '20,20': 0 });
  });
  it('no anuncia borrado correcto al fallar la flash; identifica funciones del firmware', () => {
    let s = line(initialState(), 'CAPS RESET_TOTAL CALIB_COMPLETA');
    expect(s.resetAvailable && s.calibrationAvailable).toBe(true);
    s = line(s, 'REINICIO_TOTAL_SOLICITADO');
    s = line(s, 'REINICIO_TOTAL_ERROR_FLASH mapa=0 parametros=1');
    expect(s.resetPending).toBe(false);
    expect(s.resetRevision).toBe(0);
    s = line(s, 'CAL_COMPLETA_SOLICITADA');
    expect(s.calibrating).toBe(true);
    s = line(s, 'CAL_COMPLETA_NO_APLICADA: regreso no verificado');
    expect(s.calibrating).toBe(false);
    s = line(s, 'CAL_STATE RUNNING');
    expect(s.calibrating).toBe(true);
    s = line(s, 'CAL_STATE IDLE');
    expect(s.calibrating).toBe(false);
  });
});
