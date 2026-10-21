import { describe, expect, it } from 'vitest';
import { editedParameterCommands } from '../parameterCommands';
import { initialState, telemetryReducer } from '../../telemetria/state';

describe('ajustes que conservan la calibración', () => {
  it('no envía referencias ni restablece ganancias que el usuario no editó', () => {
    expect(editedParameterCommands({}, '')).toEqual([]);
    expect(editedParameterCommands({ sideEnabled: '1' }, '')).toEqual(['set sideEnabled 1']);
    expect(editedParameterCommands({ sideAheadMm: '-12', sideLateralMm: '33' }, '')).toEqual([
      'set sideAheadMm -12',
      'set sideLateralMm 33',
    ]);
  });
  it('rechaza ajustes fuera de límites y metas incompletas sin producir órdenes', () => {
    expect(() => editedParameterCommands({ sideEnabled: '0.5' }, '')).toThrow();
    expect(() => editedParameterCommands({ sideLeftOffsetMm: '-61' }, '')).toThrow();
    expect(() => editedParameterCommands({ kpRot: '' }, '')).toThrow();
    expect(() => editedParameterCommands({}, '32,2')).toThrow();
    expect(() => editedParameterCommands({}, '4,')).toThrow();
    expect(editedParameterCommands({}, '4,2')).toEqual(['G4,2']);
  });
  it('lee valores del robot y refleja el centrado sin tocar las muestras de pose', () => {
    let state = telemetryReducer(initialState(), { type: 'line', line: 'kSteer=0.180', t: 10 });
    state = telemetryReducer(state, { type: 'line', line: 'sideEnabled=1', t: 11 });
    expect(state.boardParams.kSteer).toBe(0.18);
    expect(state.sideControl).toBe(true);
    expect(state.last).toBeNull();
    state = telemetryReducer(state, { type: 'line', line: 'OK kSteer=0.190', t: 12 });
    expect(state.boardParams.kSteer).toBe(0.19);
    state = telemetryReducer(state, { type: 'line', line: 'OK sideEnabled=0', t: 13 });
    expect(state.sideControl).toBe(false);
    state = telemetryReducer(state, { type: 'line', line: 'REINICIO', t: 14 });
    expect(state.boardParams).toEqual({});
  });
});
