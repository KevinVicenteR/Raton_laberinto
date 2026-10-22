import type { RobotSample } from '../telemetria/types';
// Mensajes de versiones antiguas del firmware (redes, riesgo, barrido): se ignoran en registros viejos.
export const isRemovedMessage = (line: string) => /^(CTRL|LOGIC|NET|RISK|SWEEP)(?:\s|$)/.test(line);
export function parseRobot(line: string, t: number): RobotSample | null {
  // NAV del firmware v2: x, y absolutos en el laberinto (mm), a = rumbo de brujula, P = paso de celda x10,
  // H = rumbo x10 (0 = este), G = tramo recto. Los registros antiguos no traen P/H/G.
  const m =
    /^(\S+) I(-?\d+) F(-?\d+) D(-?\d+) v(-?\d+) r(-?\d+) p(-?\d+)(?: x(-?\d+) y(-?\d+) a(-?\d+))?(?: P(\d+) H(-?\d+) G(\d))?(?: t(\d+))?(?: q(-?\d+),(-?\d+),(-?\d+))?(?: c(\d+),(\d+),(\d+)(?: g\d+)?)?$/.exec(
      line,
    );
  if (!m) return null;
  return {
    t,
    state: m[1],
    I: +m[2],
    F: +m[3],
    D: +m[4],
    v: +m[5],
    r: +m[6],
    p: +m[7],
    x: +(m[8] ?? 0),
    y: +(m[9] ?? 0),
    a: +(m[10] ?? 0),
    hasPose: m[8] !== undefined,
    pitch: m[11] === undefined ? undefined : +m[11] / 10,
    straight: m[13] === undefined ? undefined : m[13] === '1',
    cycle: m[14] === undefined ? undefined : +m[14],
    raw: m[15] === undefined ? undefined : m.slice(15, 18).map(Number),
    confidence: m[18] === undefined ? undefined : m.slice(18, 21).map(Number),
  };
}
