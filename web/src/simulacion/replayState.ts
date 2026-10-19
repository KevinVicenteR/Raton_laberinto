import type { ReplayData } from '../telemetria/types';
import { competition } from '../mapa/competition';
import { mazeFromReplay } from '../mapa/maze';
import { initialState, telemetryReducer } from '../telemetria/state';
export class ReplayProjector {
  private state = initialState();
  private index = 0;
  private time = -1;
  constructor(private data: ReplayData) {}
  at(time: number) {
    const data = this.data;
    if (time < this.time) {
      this.state = initialState();
      this.index = 0;
    }
    this.time = time;
    const maze = mazeFromReplay(data, competition);
    let state = this.state;
    const origin = [(maze.start[0] + 0.5) * maze.PITCH, (maze.start[1] + 0.5) * maze.PITCH - 33];
    for (let i = this.index; i < data.trace.length; i++) {
      const f = data.trace[i];
      if (f.t > time && i > 0) break;
      const motion = data.motions
          ?.filter((m) => m.t <= f.t && m.end >= f.t)
          .sort((a, b) => a.end - a.t - (b.end - b.t))[0],
        previous = data.trace[Math.max(0, i - 1)],
        speed = ((f.encoder_mm ?? 0) - (previous.encoder_mm ?? 0)) / Math.max(0.001, f.t - previous.t);
      const pose = f.truth
        ? ` x${Math.round(f.truth[0] - origin[0])} y${Math.round(f.truth[1] - origin[1])} a${Math.round((f.truth[2] * 180) / Math.PI - 90)}`
        : '';
      const line =
        f._line ??
        `${motion?.kind ?? 'REPRODUCCION'} I${Math.round(f.filtered.L)} F${Math.round(f.filtered.F)} D${Math.round(f.filtered.R)} v${Math.round(speed)} r${f.truth ? Math.round((f.truth[2] * 180) / Math.PI - 90) : 0} p${maze.PITCH - 15}${pose}`;
      state = telemetryReducer(state, { type: 'line', line, t: f.t * 1000 });
      this.index = i + 1;
      if (!f._line) {
        const raw = f.raw ? [f.raw.L, f.raw.F, f.raw.R].map(Math.round).join(',') : '-1,-1,-1',
          age = f.sensors?.map((s) => Math.round(Number(s[3]))).join(',') ?? '0,0,0';
        state = telemetryReducer(state, {
          type: 'line',
          line: `OBS raw=${raw} age=${age} phase=${motion?.kind ?? 'REPRODUCCION'} drops=0`,
          t: f.t * 1000,
        });
        if (motion)
          state = telemetryReducer(state, {
            type: 'line',
            line: `MOTION completed=${motion.completed ? 1 : 0} reason=0 actual=${Math.round(motion.actual_mm ?? 0)} error_cdeg=${Math.round((motion.angular_error ?? 0) * 100)}`,
            t: f.t * 1000,
          });
      }
    }
    this.state = state;
    return state;
  }
}
export function stateAt(data: ReplayData, time: number) {
  return new ReplayProjector(data).at(time);
}
