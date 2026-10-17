import { frame } from '../domain/protocol';
export interface Transport {
  write(text: string): Promise<void>;
}
export class CommandQueue {
  private sequence = 0;
  private tail: Promise<boolean> = Promise.resolve(true);
  private pending?: {
    seq: number;
    resolve: (reply: boolean | null) => void;
  };
  private generation = 0;
  retries = 0;
  acked = 0;
  constructor(
    private transport: Transport,
    private timeout = 800,
    private maxRetries = 3,
  ) {}
  acknowledge(line: string): boolean {
    const match = /^(ACK|NAK) (\d+)$/.exec(line);
    if (!match) return false;
    if (this.pending?.seq === +match[2]) this.pending.resolve(match[1] === 'ACK');
    return true;
  }
  cancel() {
    this.generation++;
    this.pending?.resolve(null);
    this.pending = undefined;
  }
  send(command: string): Promise<boolean> {
    const cmd = command.trim();
    if (!cmd || /[\r\n]/.test(cmd)) return Promise.resolve(false);
    if (cmd === 'x')
      return this.transport.write('x').then(
        () => true,
        () => false,
      );
    const generation = this.generation;
    const run = async () => {
      if (generation !== this.generation) return false;
      const seq = (this.sequence = (this.sequence + 1) & 65535);
      for (let attempt = 0; attempt <= this.maxRetries; attempt++) {
        if (generation !== this.generation) return false;
        let timer: ReturnType<typeof setTimeout> | undefined;
        const reply = new Promise<boolean | null>((resolve) => {
          this.pending = { seq, resolve };
          timer = setTimeout(() => resolve(null), this.timeout);
        });
        try {
          await this.transport.write(frame(seq, cmd) + '\n');
          const ok = await reply;
          if (ok) {
            this.acked++;
            return true;
          }
        } catch {
          return false;
        } finally {
          clearTimeout(timer);
          this.pending = undefined;
        }
        if (attempt < this.maxRetries) this.retries++;
      }
      return false;
    };
    this.tail = this.tail.catch(() => false).then(run);
    return this.tail;
  }
}
