import { validateReplay } from '../conexion/protocol';
import type { ReplayData, ReplaySample } from '../telemetria/types';
export class ReplaySession {
  readonly data: ReplayData;
  time = 0;
  playing = false;
  speed = 2;
  constructor(data: unknown) {
    this.data = validateReplay(data);
  }
  get duration() {
    return this.data.trace.at(-1)!.t;
  }
  seek(t: number) {
    this.time = Math.max(0, Math.min(this.duration, t));
  }
  advance(seconds: number) {
    if (this.playing) {
      this.seek(this.time + seconds * this.speed);
      if (this.time >= this.duration) this.playing = false;
    }
  }
  get index() {
    let lo = 0,
      hi = this.data.trace.length;
    while (lo < hi) {
      const mid = (lo + hi) >>> 1;
      if (this.data.trace[mid].t <= this.time) lo = mid + 1;
      else hi = mid;
    }
    return Math.max(0, lo - 1);
  }
  get sample(): ReplaySample {
    return this.data.trace[this.index];
  }
}
