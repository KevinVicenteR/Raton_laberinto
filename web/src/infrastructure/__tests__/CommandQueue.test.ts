import { afterEach, describe, it, expect, vi } from 'vitest';
import { CommandQueue, type Transport } from '../CommandQueue';
import { unframe } from '../../domain/protocol';
class FakeTransport implements Transport {
  writes: string[] = [];
  onWrite?: (text: string) => void;
  write(text: string) {
    this.writes.push(text);
    this.onWrite?.(text);
    return Promise.resolve();
  }
}
afterEach(() => vi.useRealTimers());
describe('Cola ACK/NAK con transporte falso', () => {
  it('serializa órdenes y acepta solo el ACK correspondiente', async () => {
    const transport = new FakeTransport(),
      queue = new CommandQueue(transport);
    transport.onWrite = (text) => {
      const f = unframe(text);
      if (f.kind === 'ok') {
        queue.acknowledge(`ACK ${f.seq + 1}`);
        queue.acknowledge(`ACK ${f.seq}`);
      }
    };
    expect(await Promise.all([queue.send('m3'), queue.send('s3')])).toEqual([true, true]);
    expect(transport.writes.map((s) => unframe(s))).toEqual([
      { kind: 'ok', seq: 1, payload: 'm3' },
      { kind: 'ok', seq: 2, payload: 's3' },
    ]);
    expect(queue.acked).toBe(2);
  });
  it('reintenta NAK usando la misma trama y secuencia para deduplicar en firmware', async () => {
    const transport = new FakeTransport(),
      queue = new CommandQueue(transport);
    transport.onWrite = (text) => {
      const f = unframe(text);
      if (f.kind === 'ok') queue.acknowledge(`${transport.writes.length < 3 ? 'NAK' : 'ACK'} ${f.seq}`);
    };
    expect(await queue.send('G4,2')).toBe(true);
    expect(new Set(transport.writes).size).toBe(1);
    expect(queue.retries).toBe(2);
  });
  it('espera 800 ms y realiza tres reintentos después del envío inicial', async () => {
    vi.useFakeTimers();
    const transport = new FakeTransport(),
      queue = new CommandQueue(transport);
    const result = queue.send('s4');
    await vi.advanceTimersByTimeAsync(799);
    expect(transport.writes).toHaveLength(1);
    await vi.advanceTimersByTimeAsync(1);
    expect(transport.writes).toHaveLength(2);
    await vi.advanceTimersByTimeAsync(2400);
    expect(await result).toBe(false);
    expect(transport.writes).toHaveLength(4);
    expect(queue.retries).toBe(3);
  });
  it('PARAR evita la cola y un fallo no bloquea las órdenes siguientes', async () => {
    vi.useFakeTimers();
    const transport = new FakeTransport(),
      queue = new CommandQueue(transport);
    const first = queue.send('s3');
    await vi.advanceTimersByTimeAsync(1);
    expect(await queue.send('x')).toBe(true);
    expect(transport.writes[1]).toBe('x');
    await vi.advanceTimersByTimeAsync(3200);
    expect(await first).toBe(false);
    transport.onWrite = (text) => {
      const f = unframe(text);
      if (f.kind === 'ok') queue.acknowledge(`ACK ${f.seq}`);
    };
    expect(await queue.send('?')).toBe(true);
  });
  it('cancela pendientes al desconectar y descarta órdenes en espera', async () => {
    vi.useFakeTimers();
    const transport = new FakeTransport(),
      queue = new CommandQueue(transport);
    const a = queue.send('m1'),
      b = queue.send('s1');
    await vi.advanceTimersByTimeAsync(1);
    queue.cancel();
    expect(await a).toBe(false);
    expect(await b).toBe(false);
    expect(transport.writes).toHaveLength(1);
  });
});
