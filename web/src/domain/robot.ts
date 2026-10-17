import type { RobotSample } from './types';
// Mensajes de versiones antiguas del firmware (redes, riesgo, barrido): se ignoran en registros viejos.
export const isRemovedMessage = (line: string) => /^(CTRL|LOGIC|NET|RISK|SWEEP)(?:\s|$)/.test(line);
export function parseRobot(line: string, t: number): RobotSample | null {
  const m =
    /^(\S+) I(-?\d+) F(-?\d+) D(-?\d+) v(-?\d+) r(-?\d+) p(-?\d+)(?: x(-?\d+) y(-?\d+) a(-?\d+))?(?: t(\d+))?(?: q(-?\d+),(-?\d+),(-?\d+))?(?: c(\d+),(\d+),(\d+)(?: g\d+)?)?$/.exec(
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
    cycle: m[11] === undefined ? undefined : +m[11],
    raw: m[12] === undefined ? undefined : m.slice(12, 15).map(Number),
    confidence: m[15] === undefined ? undefined : m.slice(15, 18).map(Number),
  };
}
