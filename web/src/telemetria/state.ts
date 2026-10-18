import { parse, type Diagnostic } from '../conexion/protocol';
import { parseRobot, isRemovedMessage } from '../telemetria/robot';
import { emptyOccupancy, updateOccupancy, type Occupancy } from '../mapa/occupancy';
import type { Cell, RobotSample } from './types';
export interface TelemetryState {
  last: RobotSample | null;
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
  last: null,
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
  if (isRemovedMessage(line)) return state;
  let next = { ...state, record: [...state.record, `${(t / 1000).toFixed(3)}\t${line}`] };
  const diagnostic = parse(line);
  if (diagnostic) return { ...next, diagnostics: { ...state.diagnostics, [diagnostic.kind]: diagnostic } };
  const cell = /^CELL x=(\d+) y=(\d+) w=(\d+)(?: d=(\d))?/.exec(line);
  if (cell)
    return {
      ...next,
      cells: { ...state.cells, [`${cell[1]},${cell[2]}`]: +cell[3] | (cell[4] === '1' ? 16 : 0) },
      learnRobot: [+cell[1], +cell[2]],
    };
  if (line === 'REINICIO')
    next = {
      ...next,
      cells: {},
      learnRobot: [20, 20],
      occupancy: emptyOccupancy(),
      trail: [],
      boardMode: 'Reiniciando',
    };
  if (line === 'MAPA_BORRADO') next = { ...next, cells: {}, learnRobot: [20, 20] };
  if (line === 'MAPA_NUEVO') next = { ...next, cells: {}, learnRobot: [20, 20] };
  if (line === 'RUTA_INICIO') next.learnRobot = [20, 20];
  if (/^(MODE|START) [1-6]$/.test(line)) next.boardMode = line;
  if (line.startsWith('DECIDE: ')) next.decision = line.slice(8);
  if (line === 'INICIO_MAPA')
    next = { ...next, occupancy: emptyOccupancy(), trail: [], samples: [], raceStart: t, raceEnd: null };
  if (line === 'META') next.raceEnd = t;
  const sample = parseRobot(line, t);
  if (sample)
    return {
      ...next,
      last: sample,
      samples: [...state.samples, sample].filter((s) => t - s.t <= 20000),
      trail: sample.hasPose ? [...state.trail, sample].slice(-20000) : state.trail,
      occupancy: updateOccupancy(state.occupancy, sample, action.range),
    };
  return { ...next, events: [...state.events, `${(t / 1000).toFixed(2)} s · ${line}`].slice(-1500) };
}
