import type { RobotSample, SensorFrame, SensorKey, SensorReadings } from './types';

export const SENSOR_MAX_AGE = 1500; // edad de la telemetria recibida, no del ciclo interno del sensor
export const SENSOR_DEFS = [
  {
    key: 'LI',
    label: 'Lateral izquierdo · 90°',
    short: 'LI',
    color: 'side-left',
    ahead: 0,
    left: 30,
    angle: 90,
  },
  { key: 'I', label: 'Diagonal izquierdo · 45°', short: 'DI', color: 'left', ahead: 30, left: 30, angle: 45 },
  { key: 'F', label: 'Frontal', short: 'F', color: 'front', ahead: 40, left: 0, angle: 0 },
  {
    key: 'D',
    label: 'Diagonal derecho · 45°',
    short: 'DD',
    color: 'right',
    ahead: 30,
    left: -30,
    angle: -45,
  },
  {
    key: 'LD',
    label: 'Lateral derecho · 90°',
    short: 'LD',
    color: 'side-right',
    ahead: 0,
    left: -30,
    angle: -90,
  },
] as const;

export function parseSensorLine(
  line: string,
  t: number,
  sample: RobotSample | null = null,
): { readings: SensorReadings; control?: boolean } | null {
  let keys: SensorKey[], values: string[];
  let control: boolean | undefined;
  const side = /^LATERALES I(-?\d+) D(-?\d+)(?: fresh=[01],[01])?(?: control=([01]))?$/.exec(line);
  const diagnostic =
    /^SENS I(-?\d+) F(-?\d+) D(-?\d+) fresh=[01],[01],[01] giro=-?\d+ encI=-?\d+ encD=-?\d+$/.exec(line);
  const test = /^PRUEBA_MM DI=(-?\d+) F=(-?\d+) DD=(-?\d+) LI=(-?\d+) LD=(-?\d+)$/.exec(line);
  if (side) {
    keys = ['LI', 'LD'];
    values = side.slice(1, 3);
    control = side[3] === undefined ? undefined : side[3] === '1';
  } else if (diagnostic) {
    keys = ['I', 'F', 'D'];
    values = diagnostic.slice(1, 4);
  } else if (test) {
    keys = ['I', 'F', 'D', 'LI', 'LD'];
    values = test.slice(1, 6);
  } else if (sample) {
    keys = ['I', 'F', 'D'];
    values = [sample.I, sample.F, sample.D].map(String);
  } else return null;
  if (!values.every((v) => Number.isSafeInteger(Number(v)))) return null;
  const readings: SensorReadings = {};
  keys.forEach((key, i) => {
    readings[key] = { mm: Number(values[i]), t };
  });
  return { readings, control };
}

export function currentReading(readings: SensorReadings, key: SensorKey, now: number) {
  const value = readings[key];
  return value && now >= value.t && now - value.t <= SENSOR_MAX_AGE ? value : undefined;
}

export function sensorPath(frames: SensorFrame[], key: SensorKey, end: number) {
  let path = '',
    pen = false,
    previousTime = -Infinity;
  for (const frame of frames) {
    if (frame.t - previousTime > SENSOR_MAX_AGE) pen = false;
    previousTime = frame.t;
    if (frame.t < end - 20000 || frame.t > end) {
      pen = false;
      continue;
    }
    const reading = currentReading(frame.readings, key, frame.t);
    if (!reading || reading.mm < 0) {
      pen = false;
      continue;
    }
    path += `${pen ? 'L' : 'M'}${35 + 555 * (1 - (end - frame.t) / 20000)},${230 - Math.min(600, reading.mm) / 3} `;
    pen = true;
  }
  return path;
}
