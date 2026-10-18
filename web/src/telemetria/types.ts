export type Cell = [number, number];
export type Sensors = {
  L: number;
  F: number;
  R: number;
};
export interface ReplaySample {
  t: number;
  filtered: Sensors;
  raw?: Sensors;
  truth?: [number, number, number];
  sensors?: (number | boolean)[][];
  encoder_mm?: number;
  _line?: string;
}
interface Motion {
  t: number;
  end: number;
  kind?: string;
  angular_error?: number;
  completed?: boolean;
  reason?: string;
  contacts?: number;
  actual_mm?: number;
}
export interface ReplayData {
  trace: ReplaySample[];
  maze?: string[];
  start?: Cell;
  goal?: Cell;
  pitch?: number;
  width?: number;
  height?: number;
  walls?: [Cell, Cell][];
  events?: [number, string, number, number][];
  motions?: Motion[];
  reached?: number | null;
  clean_arrival?: boolean;
  failure?: string;
  mode?: number;
  mapped_guard?: boolean;
}
export interface RobotSample {
  t: number;
  state: string;
  I: number;
  F: number;
  D: number;
  v: number;
  r: number;
  p: number;
  x: number;
  y: number;
  a: number;
  cycle?: number;
  raw?: number[];
  confidence?: number[];
  hasPose: boolean;
}
export interface Maze {
  W: number;
  H: number;
  map: string[];
  start: Cell;
  goal: Cell;
  PITCH: number;
}
interface Result {
  corridor_mm?: number;
  maze: number;
  noise_seed: number;
  mode: number;
  controller: string;
  suite: string;
  reached: number | null;
  contacts: number;
  clean_arrival: boolean;
  failure?: string;
  elapsed: number;
}
export interface Catalog {
  generated_at: string;
  run_id: string;
  objective_met: boolean;
  default_controller: string;
  controller_labels: Record<string, string>;
  results: Result[];
  replays: {
    id: string;
    label: string;
    summary: string;
    file: string;
  }[];
}
