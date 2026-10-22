import { parse, type Diagnostic } from '../conexion/protocol';
import { parseRobot, isRemovedMessage } from '../telemetria/robot';
import { emptyOccupancy, updateOccupancy, type Occupancy } from '../mapa/occupancy';
import type { Cell, RobotSample, SensorReadings, SensorFrame } from './types';
import { parseSensorLine } from './sensorData';
export interface TelemetryState {
  firmwareV3: boolean;
  syncing: boolean;
  historyOverwritten: number;
  historyLines: string[];
  last: RobotSample | null;
  sensorReadings: SensorReadings;
  sensorHistory: SensorFrame[];
  sideControl?: boolean;
  boardParams: Record<string, number>;
  samples: RobotSample[];
  trail: RobotSample[];
  cells: Record<string, number>;
  learnRobot: Cell;
  diagnostics: Record<string, Diagnostic>;
  events: string[];
  record: string[];
  decision: string;
  boardMode: string;
  occupancy: Occupancy;
  raceStart: number | null;
  raceEnd: number | null;
}
export const initialState = (): TelemetryState => ({
  firmwareV3: false,
  syncing: false,
  historyOverwritten: 0,
  historyLines: [],
  last: null,
  sensorReadings: {},
  sensorHistory: [],
  boardParams: {},
  samples: [],
  trail: [],
  cells: {},
  learnRobot: [20, 20],
  diagnostics: {},
  events: [],
  record: [],
  decision: '—',
  boardMode: 'Sin confirmar',
  occupancy: emptyOccupancy(),
  raceStart: null,
  raceEnd: null,
});
type Action =
  | {
      type: 'line';
      line: string;
      t: number;
      range?: number;
    }
  | {
      type: 'reset';
    }
  | {
      type: 'clearMap';
    };
export function telemetryReducer(state: TelemetryState, action: Action): TelemetryState {
  if (action.type === 'reset') return initialState();
  if (action.type === 'clearMap') return { ...state, occupancy: emptyOccupancy(), trail: [] };
  const { line, t } = action;
  if (line === 'FIRMWARE autonomia-v3') return { ...state, firmwareV3: true };
  if (line === 'SYNC_BEGIN') return { ...state, syncing: true, cells: {} };
  if (line.startsWith('SYNC_DONE'))
    return { ...state, syncing: false, historyOverwritten: Number(/overwritten=(\d+)/.exec(line)?.[1] ?? 0) };
  if (line.startsWith('HISTORY ')) {
    if (state.historyLines.includes(line)) return state;
    return {
      ...state,
      historyLines: [...state.historyLines, line].slice(-64),
      events: [...state.events, line].slice(-1500),
    };
  }
  const synced = /^SYNC_CELL x=(\d+) y=(\d+) w=(\d+)$/.exec(line);
  if (synced) {
    const key = synced[1] + ',' + synced[2];
    return state.cells[key] === undefined
      ? { ...state, cells: { ...state.cells, [key]: +synced[3] } }
      : state;
  }
  if (isRemovedMessage(line)) return state;
  let next: TelemetryState = {
    ...state,
    record: [...state.record, `${(t / 1000).toFixed(3)}\t${line}`].slice(-100000),
  };
  const diagnostic = parse(line);
  if (diagnostic)
    return {
      ...next,
      diagnostics: { ...state.diagnostics, [diagnostic.kind]: diagnostic },
      decision:
        diagnostic.kind === 'AUTO'
          ? String(diagnostic.phase) + ' · ' + String(diagnostic.reason ?? '')
          : state.decision,
      events:
        diagnostic.kind === 'AUTO'
          ? [...state.events, `${(t / 1000).toFixed(2)} s · ${line}`].slice(-1500)
          : state.events,
      raceEnd:
        diagnostic.kind === 'AUTO' && ['COMPLETO', 'PARADO'].includes(String(diagnostic.phase))
          ? t
          : state.raceEnd,
    };
  const cell = /^CELL x=(\d+) y=(\d+) w=(\d+)(?: d=(\d))?/.exec(line);
  if (cell) {
    const key = `${cell[1]},${cell[2]}`;
    const mask = +cell[3] | (cell[4] === '1' ? 16 : 0);
    return {
      ...next,
      cells: state.cells[key] === mask ? state.cells : { ...state.cells, [key]: mask },
      learnRobot: [+cell[1], +cell[2]],
    };
  }
  if (line === 'REINICIO')
    next = {
      ...next,
      cells: {},
      learnRobot: [20, 20],
      occupancy: emptyOccupancy(),
      trail: [],
      boardMode: 'Reiniciando',
      sensorReadings: {},
      sensorHistory: [],
      sideControl: undefined,
      boardParams: {},
      firmwareV3: false,
    };
  if (line === 'MAPA_BORRADO') next = { ...next, cells: {}, learnRobot: [20, 20] };
  if (line === 'MAPA_NUEVO') next = { ...next, cells: {}, learnRobot: [20, 20] };
  if (line === 'RUTA_INICIO') next.learnRobot = [20, 20];
  if (/^(MODE|START) [1-7]$/.test(line)) next.boardMode = line;
  if (line.startsWith('PRUEBA_INICIO')) next.boardMode = 'PRUEBA SENSORES';
  if (line.startsWith('PRUEBA_FIN')) next.boardMode = 'Esperando';
  if (line.startsWith('DIAGNOSTICO_SENSORES')) next.boardMode = 'DIAGNOSTICO SENSORES';
  if (line === 'FIN_DIAGNOSTICO_SENSORES') next.boardMode = 'Esperando';
  if (line.startsWith('DECIDE: ')) next.decision = line.slice(8);
  if (line === 'INICIO_MAPA')
    next = {
      ...next,
      occupancy: emptyOccupancy(),
      trail: [],
      samples: [],
      sensorReadings: {},
      sensorHistory: [],
      raceStart: t,
      raceEnd: null,
    };
  if (line === 'META') next.raceEnd = t;
  const parameter = /^(?:OK )?([A-Za-z][A-Za-z0-9]*)=(-?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?)$/.exec(line);
  if (parameter && Number.isFinite(+parameter[2])) {
    next.boardParams = { ...state.boardParams, [parameter[1]]: +parameter[2] };
    if (parameter[1] === 'sideEnabled') next.sideControl = +parameter[2] !== 0;
  }
  const sample = parseRobot(line, t);
  const sensors = parseSensorLine(line, t, sample);
  if (sensors) {
    next.sensorReadings = { ...next.sensorReadings, ...sensors.readings };
    next.sensorHistory = [...next.sensorHistory, { t, readings: next.sensorReadings }].filter(
      (f) => t - f.t <= 20000,
    );
    if (sensors.control !== undefined) next.sideControl = sensors.control;
  }
  const control = /^LATERALES izq=.+ der=.+ control=([01])$/.exec(line);
  if (control) next.sideControl = control[1] === '1';
  if (sample)
    return {
      ...next,
      last: sample,
      samples: [...state.samples, sample].filter((s) => t - s.t <= 20000),
      trail: sample.hasPose ? [...state.trail, sample].slice(-20000) : state.trail,
      occupancy: updateOccupancy(
        state.occupancy,
        sample,
        action.range,
        next.sensorReadings,
        state.boardParams,
      ),
    };
  return { ...next, events: [...state.events, `${(t / 1000).toFixed(2)} s · ${line}`].slice(-1500) };
}
