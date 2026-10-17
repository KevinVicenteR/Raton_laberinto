import { unframe } from '../domain/protocol';
import type { Transport } from './CommandQueue';
const NUS = '6e400001-b5a3-f393-e0a9-e50e24dcca9e';
const RX = '6e400002-b5a3-f393-e0a9-e50e24dcca9e',
  TX = '6e400003-b5a3-f393-e0a9-e50e24dcca9e';
export class BleTransport implements Transport {
  private device?: BluetoothDevice;
  private rx?: BluetoothRemoteGATTCharacteristic;
  private tx?: BluetoothRemoteGATTCharacteristic;
  private buffer = '';
  private decoder = new TextDecoder();
  private lastSeq?: number;
  private resetPending = false;
  private generation = 0;
  stats = { ok: 0, bad: 0, lost: 0, duplicates: 0 };
  onLine: (line: string) => void = () => {};
  onConnection: (connected: boolean, name: string) => void = () => {};
  onError: (message: string) => void = () => {};
  get connected() {
    return !!this.rx && !!this.device?.gatt?.connected;
  }
  async connect() {
    this.generation++;
    this.resetPending = false;
    this.disconnect();
    if (!navigator.bluetooth)
      throw Error('Web Bluetooth no está disponible. Usa Chrome o Edge con HTTPS o localhost.');
    this.device = await navigator.bluetooth.requestDevice({
      filters: [{ services: [NUS] }],
      optionalServices: [NUS],
    });
    this.device.addEventListener('gattserverdisconnected', this.disconnected);
    await this.open();
  }
  private async open() {
    try {
      const server = await this.device!.gatt!.connect();
      const service = await server.getPrimaryService(NUS);
      const rx = await service.getCharacteristic(RX),
        tx = await service.getCharacteristic(TX);
      this.tx?.removeEventListener('characteristicvaluechanged', this.notify);
      this.tx = tx;
      tx.addEventListener('characteristicvaluechanged', this.notify);
      await tx.startNotifications();
      this.rx = rx;
      this.buffer = '';
      this.lastSeq = undefined;
      this.onConnection(true, this.device?.name ?? 'Ratón');
    } catch (error) {
      this.rx = undefined;
      this.device?.gatt?.disconnect();
      throw error;
    }
  }
  private notify = (event: Event) => {
    const value = (event.target as BluetoothRemoteGATTCharacteristic).value;
    if (value) this.feed(this.decoder.decode(value, { stream: true }));
  };
  feed(chunk: string) {
    this.buffer += chunk;
    let index;
    while ((index = this.buffer.indexOf('\n')) >= 0) {
      const raw = this.buffer.slice(0, index).trim();
      this.buffer = this.buffer.slice(index + 1);
      if (!raw) continue;
      const f = unframe(raw);
      if (f.kind === 'bad') {
        this.stats.bad++;
        continue;
      }
      if (f.kind === 'ok') {
        if (this.lastSeq !== undefined) {
          const delta = (f.seq - this.lastSeq + 65536) % 65536;
          if (delta === 0 || delta > 32768) {
            this.stats.duplicates++;
            continue;
          }
          this.stats.lost += delta - 1;
        }
        this.lastSeq = f.seq;
        this.stats.ok++;
      }
      if (f.payload === 'REINICIO') this.resetPending = true;
      this.onLine(f.payload);
    }
    if (this.buffer.length > 65536) {
      this.buffer = '';
      this.stats.bad++;
    }
  }
  private disconnected = () => {
    this.rx = undefined;
    this.onConnection(false, '');
    if (this.resetPending) {
      this.resetPending = false;
      void this.reconnect(this.generation);
    }
  };
  private async reconnect(generation: number) {
    for (let i = 0; i < 10; i++) {
      await new Promise((r) => setTimeout(r, 1500));
      if (generation !== this.generation) return;
      try {
        await this.open();
        return;
      } catch {
        /* board still rebooting */
      }
    }
    this.onError('No se pudo reconectar tras reiniciar. Pulsa Conectar.');
  }
  async write(text: string) {
    if (!this.rx || !this.connected) throw Error('Robot desconectado');
    const bytes = new TextEncoder().encode(text);
    for (let i = 0; i < bytes.length; i += 20) await this.rx.writeValue(bytes.slice(i, i + 20));
  }
  disconnect() {
    this.generation++;
    this.resetPending = false;
    this.device?.removeEventListener('gattserverdisconnected', this.disconnected);
    this.tx?.removeEventListener('characteristicvaluechanged', this.notify);
    this.device?.gatt?.disconnect();
    this.rx = undefined;
    this.tx = undefined;
    this.onConnection(false, '');
  }
}
