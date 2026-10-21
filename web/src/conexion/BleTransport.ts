import { unframe } from './protocol';
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
  private generation = 0;
  private wasConnected = false;
  private reconnectingGeneration?: number;
  stats = { ok: 0, bad: 0, lost: 0, duplicates: 0 };
  onLine: (line: string) => void = () => {};
  onConnection: (connected: boolean, name: string) => void = () => {};
  onError: (message: string) => void = () => {};
  get connected() {
    return !!this.rx && !!this.device?.gatt?.connected;
  }
  async connect() {
    this.generation++;
    this.disconnect();
    if (!navigator.bluetooth)
      throw Error('Web Bluetooth no está disponible. Usa Chrome o Edge con HTTPS o localhost.');
    this.device = await navigator.bluetooth.requestDevice({
      filters: [{ services: [NUS] }],
      optionalServices: [NUS],
    });
    this.device.addEventListener('gattserverdisconnected', this.disconnected);
    await this.openWithRetries(this.generation, 3);
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
      this.wasConnected = true;
      this.onConnection(true, this.device?.name ?? 'Ratón');
    } catch (error) {
      this.rx = undefined;
      this.device?.gatt?.disconnect();
      throw error;
    }
  }
  private async openWithRetries(generation: number, attempts: number) {
    let lastError: unknown;
    for (let attempt = 0; attempt < attempts; attempt++) {
      if (generation !== this.generation) throw Error('Conexión cancelada.');
      try {
        await this.open();
        return;
      } catch (error) {
        lastError = error;
        if (attempt + 1 < attempts)
          await new Promise((resolve) => setTimeout(resolve, 400 * (attempt + 1)));
      }
    }
    throw lastError;
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
      this.onLine(f.payload);
    }
    if (this.buffer.length > 65536) {
      this.buffer = '';
      this.stats.bad++;
    }
  }
  private disconnected = () => {
    if (this.device?.gatt?.connected) return;
    this.rx = undefined;
    this.onConnection(false, '');
    if (this.wasConnected && this.reconnectingGeneration !== this.generation)
      void this.reconnect(this.generation);
  };
  private async reconnect(generation: number) {
    if (this.reconnectingGeneration === generation) return;
    this.reconnectingGeneration = generation;
    try {
      for (let i = 0; i < 10; i++) {
        await new Promise((r) => setTimeout(r, 1500));
        if (generation !== this.generation) return;
        try {
          await this.open();
          return;
        } catch {
          // La placa puede estar reiniciando o recuperando el enlace.
        }
      }
      this.onError('Se perdió la conexión y no se pudo recuperar. Pulsa Conectar.');
    } finally {
      if (this.reconnectingGeneration === generation) this.reconnectingGeneration = undefined;
    }
  }
  async write(text: string) {
    if (!this.rx || !this.connected) throw Error('Robot desconectado');
    const bytes = new TextEncoder().encode(text);
    for (let i = 0; i < bytes.length; i += 20) await this.rx.writeValue(bytes.slice(i, i + 20));
  }
  disconnect() {
    this.generation++;
    this.wasConnected = false;
    this.device?.removeEventListener('gattserverdisconnected', this.disconnected);
    this.tx?.removeEventListener('characteristicvaluechanged', this.notify);
    this.device?.gatt?.disconnect();
    this.rx = undefined;
    this.tx = undefined;
    this.onConnection(false, '');
  }
}
