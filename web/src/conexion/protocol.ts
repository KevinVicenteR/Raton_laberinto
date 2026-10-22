import type { ReplayData } from '../telemetria/types';
type Field = number | string | number[];
export type Diagnostic = {
  kind: string;
} & Record<string, Field>;
type Frame =
  | {
      kind: 'plain';
      payload: string;
    }
  | {
      kind: 'ok';
      seq: number;
      payload: string;
    }
  | {
      kind: 'bad';
    };
export function parse(line: string): Diagnostic | null {
  line = String(line)
    .trim()
    .replace(/^\d+(?:\.\d+)?\t/, '');
  const head = line.split(/\s+/, 1)[0];
  if (!['OBS', 'MOTION', 'AUTO', 'HEALTH', 'POSE', 'CONTROL'].includes(head)) return null;
  const fields: Record<string, Field> = {};
  for (const match of line.matchAll(/([a-zA-Z_]+)=([^\s]+)/g)) {
    const parts = match[2].split(',');
    fields[match[1]] =
      parts.length > 1
        ? parts.map((v) => Number(v))
        : /^-?\d+(?:\.\d+)?$/.test(parts[0])
          ? Number(parts[0])
          : parts[0];
  }
  for (const value of Object.values(fields)) {
    if (typeof value === 'number' && !Number.isFinite(value)) return null;
    if (Array.isArray(value) && value.some((v) => !Number.isFinite(v))) return null;
  }
  for (const key of ['raw', 'filtered', 'age', 'conf', 'enc']) {
    if (
      key in fields &&
      (!Array.isArray(fields[key]) ||
        fields[key].length !== (key === 'enc' ? 2 : head === 'HEALTH' && key === 'age' ? 5 : 3))
    )
      return null;
  }
  for (const [key, value] of Object.entries(fields)) {
    if (
      !['phase', 'pick', ...(head === 'AUTO' || head === 'HEALTH' ? ['reason'] : [])].includes(key) &&
      !Array.isArray(value) &&
      typeof value !== 'number'
    )
      return null;
  }
  return { ...fields, kind: head };
}
export function validateReplay(input: unknown): ReplayData {
  const data = input as ReplayData;
  if (!data || !Array.isArray(data.trace) || !data.trace.length || data.trace.length > 100000)
    throw Error('La sesión debe contener entre 1 y 100000 muestras.');
  let previous = -Infinity;
  for (const frame of data.trace) {
    if (
      !Number.isFinite(frame.t) ||
      frame.t < 0 ||
      frame.t < previous ||
      !frame.filtered ||
      !(['L', 'F', 'R'] as const).every((k) => Number.isFinite(frame.filtered[k]))
    )
      throw Error('Muestras incompletas o tiempos desordenados.');
    if (
      frame.truth &&
      (!Array.isArray(frame.truth) ||
        frame.truth.length !== 3 ||
        frame.truth.some((v) => !Number.isFinite(v)))
    )
      throw Error('La posición guardada no es válida.');
    if (frame.raw && !(['L', 'F', 'R'] as const).every((k) => Number.isFinite(frame.raw![k])))
      throw Error('Lecturas crudas inválidas.');
    if (
      frame.sensors &&
      (!Array.isArray(frame.sensors) ||
        frame.sensors.length !== 3 ||
        frame.sensors.some(
          (s) =>
            !Array.isArray(s) ||
            s.length !== 4 ||
            !s.every((v) => typeof v === 'boolean' || Number.isFinite(v)),
        ))
    )
      throw Error('Observaciones inválidas.');
    if (frame.encoder_mm !== undefined && !Number.isFinite(frame.encoder_mm))
      throw Error('Encoder inválido.');
    previous = frame.t;
  }
  if (
    data.maze &&
    (!Array.isArray(data.maze) ||
      data.maze.length % 2 !== 1 ||
      data.maze.length > 65 ||
      data.maze.some((row) => typeof row !== 'string' || row.length > 133))
  )
    throw Error('Mapa no válido.');
  if (data.trace.some((f) => f.truth) && !data.trace.every((f) => f.truth))
    throw Error('La posición debe estar presente en todas las muestras o en ninguna.');
  if (data.maze) {
    if (!data.start || !data.goal) throw Error('El mapa requiere salida y meta.');
    const height = (data.maze.length - 1) / 2,
      width = (data.maze[0].length - 1) / 4;
    if (
      height < 1 ||
      !Number.isInteger(width) ||
      width < 1 ||
      data.maze.some((row) => row.length !== data.maze![0].length)
    )
      throw Error('Dimensiones del mapa inválidas.');
    for (const key of ['start', 'goal'] as const)
      if (
        data[key] &&
        (!Array.isArray(data[key]) ||
          data[key].length !== 2 ||
          !data[key].every(Number.isInteger) ||
          data[key][0] < 0 ||
          data[key][0] >= width ||
          data[key][1] < 0 ||
          data[key][1] >= height)
      )
        throw Error('Celda fuera del mapa.');
  }
  if (
    data.events &&
    (!Array.isArray(data.events) ||
      data.events.length > 20000 ||
      data.events.some(
        (e) =>
          !Array.isArray(e) ||
          e.length < 4 ||
          !Number.isFinite(e[0]) ||
          typeof e[1] !== 'string' ||
          !Number.isFinite(e[2]) ||
          !Number.isFinite(e[3]),
      ))
  )
    throw Error('Eventos inválidos.');
  if (
    data.motions &&
    (!Array.isArray(data.motions) ||
      data.motions.length > 20000 ||
      data.motions.some((m) => !Number.isFinite(m.t) || !Number.isFinite(m.end) || m.end < m.t))
  )
    throw Error('Maniobras inválidas.');
  if (
    data.walls &&
    (!Array.isArray(data.walls) ||
      data.walls.length > 20000 ||
      data.walls.some(
        (w) =>
          !Array.isArray(w) ||
          w.length !== 2 ||
          w.some((p) => !Array.isArray(p) || p.length !== 2 || !p.every(Number.isFinite)),
      ))
  )
    throw Error('Paredes inválidas.');
  for (const key of ['width', 'height', 'pitch'] as const)
    if (data[key] !== undefined && (!Number.isFinite(data[key]) || data[key] <= 0))
      throw Error('Escala inválida.');
  if (
    data.reached !== undefined &&
    data.reached !== null &&
    (!Number.isFinite(data.reached) || data.reached < 0)
  )
    throw Error('Llegada inválida.');
  if (data.clean_arrival !== undefined && typeof data.clean_arrival !== 'boolean')
    throw Error('Resultado inválido.');
  if (
    data.clean_arrival &&
    (!Number.isFinite(data.reached) || (data.events || []).some((e) => e[1] === 'choque'))
  )
    throw Error('La llegada limpia contradice el registro.');
  return data;
}
// Protocolo de lineas (igual que lib/raton_core/src/frame.cpp): "$<seq>,<datos>*<CRC8 hex>".
export function crc8(text: string) {
  let crc = 0;
  for (let i = 0; i < text.length; i++) {
    crc ^= text.charCodeAt(i) & 255;
    for (let b = 0; b < 8; b++) crc = crc & 128 ? ((crc << 1) ^ 7) & 255 : (crc << 1) & 255;
  }
  return crc;
}
export function frame(seq: number, payload: string) {
  const body = `${seq & 65535},${payload}`;
  return `$${body}*${crc8(body).toString(16).toUpperCase().padStart(2, '0')}`;
}
// {kind:'plain',payload} | {kind:'ok',seq,payload} | {kind:'bad'} (CRC malo o mal formada)
export function unframe(line: string): Frame {
  line = String(line).trim();
  if (line[0] !== '$') return { kind: 'plain', payload: line };
  const m = /^\$(\d{1,5}),(.*)\*([0-9A-Fa-f]{2})$/.exec(line);
  if (!m || +m[1] > 65535) return { kind: 'bad' };
  if (crc8(`${m[1]},${m[2]}`) !== parseInt(m[3], 16)) return { kind: 'bad' };
  return { kind: 'ok', seq: +m[1], payload: m[2] };
}
