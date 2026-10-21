import type { RobotSample } from '../telemetria/types';
export interface Supervisor {
  history: RobotSample[];
  sides: number[];
  contacts: number[];
  wall?: number;
  dead?: number;
  lastAction: number;
  lastChange: number;
  lastCalm: number;
  speed: number;
  message: string;
}
export const initialSupervisor = (speed = 100): Supervisor => ({
  history: [],
  sides: [],
  contacts: [],
  lastAction: 0,
  lastChange: 0,
  lastCalm: 0,
  speed,
  message: 'Vigilando',
});
const median = (v: number[]) => [...v].sort((a, b) => a - b)[Math.floor(v.length / 2)];
export function supervise(
  prev: Supervisor,
  s: RobotSample,
  _mode: number,
  enabled: boolean,
  learn: boolean,
  userSpeed: number,
): {
  state: Supervisor;
  commands: string[];
} {
  const state = {
      ...prev,
      history: [...prev.history, s].filter((h) => s.t - h.t <= 30000),
      sides: [...prev.sides],
      contacts: [...prev.contacts],
    },
    commands: string[] = [];
  const face = (mm: number) => mm * Math.sin((31 * Math.PI) / 180) + 30 - 40;
  if (s.state === 'RECTA' && s.v > 20 && s.I >= 0 && s.D >= 0 && face(s.I) + face(s.D) < 200)
    state.sides.push(Math.max(face(s.I), face(s.D)));
  state.sides = state.sides.slice(-400);
  if (
    prev.history.at(-1)?.state !== s.state &&
    ['CHOQUE', 'ATASCO'].includes(s.state) &&
    s.F >= 0 &&
    s.F < 120
  )
    state.contacts.push(s.F);
  state.contacts = state.contacts.slice(-20);
  if (state.sides.length >= 60) {
    const w = Math.max(40, Math.min(100, Math.round(median(state.sides) + 20)));
    if (learn && (state.wall === undefined || Math.abs(w - state.wall) >= 10)) {
      commands.push(`w${w}`);
      state.wall = w;
    }
  }
  if (state.contacts.length >= 3) {
    const z = Math.max(20, Math.min(120, Math.round(median(state.contacts))));
    if (learn && (state.dead === undefined || Math.abs(z - state.dead) >= 5)) {
      commands.push(`z${z}`);
      state.dead = z;
    }
  }
  const recent = state.history.filter((h) => s.t - h.t < 12000);
  if (
    !enabled ||
    ['PARADO', 'ESPERA', 'META', 'FALLO'].includes(s.state) ||
    s.t - state.lastAction < 4000 ||
    recent.length < 20
  )
    return { state, commands };
  const cx = recent.reduce((n, h) => n + h.x, 0) / recent.length,
    cy = recent.reduce((n, h) => n + h.y, 0) / recent.length;
  const turning = recent.filter((h, i) => i && h.state === 'GIRO' && recent[i - 1].state !== 'GIRO').length;
  const hits = state.history.filter(
    (h, i) =>
      i &&
      s.t - h.t < 15000 &&
      h.t > state.lastChange &&
      ['CHOQUE', 'ROCE', 'ATASCO', 'DESATASCO'].includes(h.state) &&
      state.history[i - 1].state !== h.state,
  ).length;
  if (Math.max(...recent.map((h) => Math.hypot(h.x - cx, h.y - cy))) < 150 && turning >= 3) {
    state.message = 'Muchos giros sin avanzar (solo aviso)';
    state.lastAction = s.t;
  } else if (hits >= 3 && state.speed > 50) {
    state.speed = Math.max(50, Math.round(state.speed * 0.8));
    commands.push(`v${state.speed}`);
    state.lastCalm = state.lastChange = state.lastAction = s.t;
    state.message = `${hits} contactos: velocidad ${state.speed} mm/s`;
  } else if (hits === 0 && state.speed < userSpeed && s.t - state.lastCalm > 20000) {
    state.speed = Math.min(userSpeed, state.speed + 10);
    commands.push(`v${state.speed}`);
    state.lastCalm = state.lastChange = state.lastAction = s.t;
    state.message = `Velocidad recuperada: ${state.speed} mm/s`;
  } else
    for (const key of ['I', 'F', 'D'] as const)
      if (recent.every((h) => h[key] < 0) && recent.some((h) => h.v > 20)) {
        state.message = `Láser ${key} sin lectura en 12 s`;
        state.lastAction = s.t;
        break;
      }
  return { state, commands };
}
