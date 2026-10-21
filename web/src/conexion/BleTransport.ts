import { unframe } from './protocol';
import type { Transport } from './CommandQueue';
const NUS = '6e400001-b5a3-f393-e0a9-e50e24dcca9e';
const RX = '6e400002-b5a3-f393-e0a9-e50e24dcca9e';
const TX = '6e400003-b5a3-f393-e0a9-e50e24dcca9e';
function preference(key: string, value?: string) {
  try {
    if (value !== undefined) localStorage.setItem(key, value);
    return localStorage.getItem(key);
  } catch {
    return null;
  }
}
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
  private pending = false;
  stats = { ok: 0, bad: 0, lost: 0, duplicates: 0 };
  onLine: (line: string) => void = () => {};
  onConnection: (connected: boolean, name: string) => void = () => {};
  onError: (message: string) => void = () => {};
  get connected() {
    return !!this.rx && !!this.device?.gatt?.connected;
  }
  get connecting() {
    return this.pending || this.reconnectingGeneration === this.generation;
  }
  async connect() {
    this.close();
    const generation = this.generation;
    if (!navigator.bluetooth) throw Error('Usa Chrome o Edge con HTTPS o localhost para Bluetooth.');
    preference('raton-auto-connect', 'on');
    this.pending = true;
    try {
      const device = await navigator.bluetooth.requestDevice({
        filters: [{ services: [NUS] }],
        optionalServices: [NUS],
      });
      if (generation !== this.generation) return;
      this.selectDevice(device);
      await this.openWithRetries(generation, 3);
    } finally {
      if (generation === this.generation) this.pending = false;
    }
  }
  // Never opens a chooser. Only the previously authorized robot can be restored.
  async autoConnect() {
    if (this.connected || this.connecting || preference('raton-auto-connect') === 'off') return false;
    if (!navigator.bluetooth?.getDevices) return false;
    const generation = this.generation;
    this.pending = true;
    try {
      const devices = await navigator.bluetooth.getDevices();
      if (generation !== this.generation) return false;
      const id = preference('raton-device');
      const named = devices.filter((d) => d.name === 'Raton');
      const device = id ? devices.find((d) => d.id === id) : named.length === 1 ? named[0] : undefined;
      if (!device) return false;
      this.selectDevice(device);
      await this.openWithRetries(generation, 3);
      return true;
    } catch (error) {
      if (generation === this.generation) this.onError('Buscando el robot autorizado: ' + String(error));
      return false;
    } finally {
      if (generation === this.generation) this.pending = false;
    }
  }
  private selectDevice(device: BluetoothDevice) {
    this.device?.removeEventListener('gattserverdisconnected', this.disconnected);
    this.device = device;
    device.addEventListener('gattserverdisconnected', this.disconnected);
  }
  private async open(generation: number) {
    const device = this.device;
    if (!device?.gatt) throw Error('Robot sin servicio Bluetooth.');
    const valid = () => generation === this.generation && this.device === device;
    try {
      const server = await device.gatt.connect();
      if (!valid()) {
        if (this.device !== device) device.gatt.disconnect();
        throw Error('Conexión cancelada.');
      }
      const service = await server.getPrimaryService(NUS);
      const rx = await service.getCharacteristic(RX);
      const tx = await service.getCharacteristic(TX);
      if (!valid()) throw Error('Conexión cancelada.');
      await tx.startNotifications();
      if (!valid()) throw Error('Conexión cancelada.');
      this.tx?.removeEventListener('characteristicvaluechanged', this.notify);
      this.tx = tx;
      tx.addEventListener('characteristicvaluechanged', this.notify);
      this.rx = rx;
      this.buffer = '';
      this.decoder = new TextDecoder();
      this.lastSeq = undefined;
      this.wasConnected = true;
      preference('raton-device', device.id);
      this.onConnection(true, device.name ?? 'Raton');
    } catch (error) {
      if (valid()) {
        this.rx = undefined;
        device.gatt.disconnect();
      }
      throw error;
    }
  }
  private async openWithRetries(generation: number, attempts: number) {
    let lastError: unknown;
    for (let attempt = 0; attempt < attempts; attempt++) {
      if (generation !== this.generation) throw Error('Conexión cancelada.');
      try {
        await this.open(generation);
        return;
      } catch (error) {
        lastError = error;
        if (generation !== this.generation) throw error;
        if (attempt + 1 < attempts) await new Promise((r) => setTimeout(r, 400 * (attempt + 1)));
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
    if (this.wasConnected && !this.pending && this.reconnectingGeneration !== this.generation)
      void this.reconnect(this.generation);
  };
  private async reconnect(generation: number) {
    this.reconnectingGeneration = generation;
    try {
      for (let i = 0; i < 10; i++) {
        await new Promise((r) => setTimeout(r, Math.min(1000 * 2 ** i, 10000)));
        if (generation !== this.generation) return;
        try {
          await this.open(generation);
          return;
        } catch {
          /* Robot reiniciando o fuera de alcance. */
        }
      }
      if (generation === this.generation)
        this.onError('Esperando al robot. La búsqueda automática sigue activa.');
    } finally {
      if (this.reconnectingGeneration === generation) this.reconnectingGeneration = undefined;
    }
  }
  async write(text: string) {
    if (!this.rx || !this.connected) throw Error('Robot desconectado');
    const bytes = new TextEncoder().encode(text);
    for (let i = 0; i < bytes.length; i += 20) await this.rx.writeValue(bytes.slice(i, i + 20));
  }
  private close() {
    this.generation++;
    this.pending = false;
    this.wasConnected = false;
    const device = this.device;
    this.device = undefined;
    device?.removeEventListener('gattserverdisconnected', this.disconnected);
    this.tx?.removeEventListener('characteristicvaluechanged', this.notify);
    device?.gatt?.disconnect();
    this.rx = undefined;
    this.tx = undefined;
    this.onConnection(false, '');
  }
  disconnect() {
    preference('raton-auto-connect', 'off');
    this.close();
  }
  dispose() {
    this.close();
  }
}
