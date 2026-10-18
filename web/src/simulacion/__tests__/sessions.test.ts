import { describe, it, expect } from 'vitest';
import { BleTransport } from '../../conexion/BleTransport';
import { frame } from '../../conexion/protocol';
import { parseRecording } from '../../telemetria/recording';
import { ReplaySession } from '../ReplaySession';
import { stateAt } from '../replayState';
describe('Recepción y reproducción', () => {
  it('recompone fragmentos, detecta pérdidas, CRC y duplicados incluyendo wrap', () => {
    const ble = new BleTransport(),
      lines: string[] = [];
    ble.onLine = (line) => lines.push(line);
    const data =
      frame(65535, 'CELL x=20 y=20 w=1') +
      '\n' +
      frame(0, 'META') +
      '\n' +
      frame(0, 'META') +
      '\n' +
      frame(2, 'START 3') +
      '\n$3,error*00\n';
    for (let i = 0; i < data.length; i += 7) ble.feed(data.slice(i, i + 7));
    expect(lines).toEqual(['CELL x=20 y=20 w=1', 'META', 'START 3']);
    expect(ble.stats).toEqual({ ok: 3, bad: 1, lost: 1, duplicates: 1 });
  });
  it('seek reconstruye eventos antiguos y no conserva celdas de tiempos futuros', () => {
    const data = parseRecording(
      'segundos\tlinea\n0\tMAPA_NUEVO\n1\tCELL x=20 y=20 w=3\n2\tDECIDE: recto\n3\tCELL x=20 y=21 w=8',
    );
    const session = new ReplaySession(data);
    session.seek(2);
    expect(stateAt(data, session.time).cells).toEqual({ '20,20': 3 });
    expect(stateAt(data, session.time).decision).toBe('recto');
    session.playing = true;
    session.advance(1);
    expect(session.time).toBe(3);
    expect(session.playing).toBe(false);
    expect(stateAt(data, 3).cells['20,21']).toBe(8);
  });
  it('rechaza TXT vacío o fuera de orden', () => {
    expect(() => parseRecording('sin muestras')).toThrow();
    expect(() => parseRecording('2\tMETA\n1\tINICIO_MAPA')).toThrow();
  });
});
