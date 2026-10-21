import { afterEach, describe, expect, it, vi } from 'vitest';
import { BleTransport } from '../BleTransport';

function mockBluetooth(failFirstConnection = false) {
  const rx = { writeValue: vi.fn(async () => {}) },
    tx = {
      addEventListener: vi.fn(),
      removeEventListener: vi.fn(),
      startNotifications: vi.fn(async () => {}),
    },
    service = {
      getCharacteristic: vi.fn(async (uuid: string) =>
        uuid.endsWith('0002-b5a3-f393-e0a9-e50e24dcca9e') ? rx : tx,
      ),
    };
  const device = new EventTarget() as BluetoothDevice;
  Object.defineProperty(device, 'name', { value: 'Raton' });
  let attempts = 0,
    connected = false;
  const gatt = {
    get connected() {
      return connected;
    },
    connect: vi.fn(async () => {
      attempts++;
      if (failFirstConnection && attempts === 1) throw Error('Conexión temporalmente no disponible');
      connected = true;
      return { getPrimaryService: vi.fn(async () => service) };
    }),
    disconnect: vi.fn(() => {
      connected = false;
      device.dispatchEvent(new Event('gattserverdisconnected'));
    }),
  } as unknown as BluetoothRemoteGATTServer;
  Object.defineProperty(device, 'gatt', { value: gatt });
  vi.stubGlobal('navigator', {
    bluetooth: { requestDevice: vi.fn(async () => device) },
  });
  return { device, gatt, service };
}

afterEach(() => {
  vi.useRealTimers();
  vi.unstubAllGlobals();
});

describe('BleTransport', () => {
  it('restaura un robot autorizado sin abrir selector', async () => {
    const { device, gatt } = mockBluetooth();
    const storage = new Map<string, string>();
    vi.stubGlobal('localStorage', {
      getItem: (k: string) => storage.get(k) ?? null,
      setItem: (k: string, v: string) => storage.set(k, v),
    });
    Object.assign(navigator.bluetooth, { getDevices: vi.fn(async () => [device]) });
    const transport = new BleTransport();
    expect(await transport.autoConnect()).toBe(true);
    expect(navigator.bluetooth.requestDevice).not.toHaveBeenCalled();
    expect(gatt.connect).toHaveBeenCalledTimes(1);
    transport.disconnect();
    expect(await transport.autoConnect()).toBe(false);
  });

  it('no adivina entre dos robots autorizados', async () => {
    const { device } = mockBluetooth();
    vi.stubGlobal('localStorage', { getItem: () => null, setItem: () => {} });
    Object.assign(navigator.bluetooth, { getDevices: vi.fn(async () => [device, device]) });
    const transport = new BleTransport();
    expect(await transport.autoConnect()).toBe(false);
    expect(navigator.bluetooth.requestDevice).not.toHaveBeenCalled();
  });

  it('una conexión tardía no revive después de desconectar', async () => {
    const { gatt, service } = mockBluetooth();
    let complete!: (value: BluetoothRemoteGATTServer) => void;
    vi.spyOn(gatt, 'connect').mockImplementationOnce(
      () =>
        new Promise((resolve) => {
          complete = resolve;
        }),
    );
    const transport = new BleTransport();
    const states: boolean[] = [];
    transport.onConnection = (on) => states.push(on);
    const pending = transport.connect();
    await Promise.resolve();
    await Promise.resolve();
    transport.disconnect();
    complete({ getPrimaryService: async () => service } as unknown as BluetoothRemoteGATTServer);
    await expect(pending).rejects.toThrow('cancelada');
    expect(states).not.toContain(true);
    expect(transport.connected).toBe(false);
  });
  it('reintenta una conexión GATT inicial fallida', async () => {
    vi.useFakeTimers();
    const { gatt } = mockBluetooth(true),
      transport = new BleTransport(),
      connecting = transport.connect();

    await vi.advanceTimersByTimeAsync(400);
    await connecting;

    expect(gatt.connect).toHaveBeenCalledTimes(2);
    expect(transport.connected).toBe(true);
    transport.disconnect();
  });

  it('recupera una desconexión inesperada', async () => {
    vi.useFakeTimers();
    const { gatt } = mockBluetooth(),
      transport = new BleTransport(),
      states: boolean[] = [];
    transport.onConnection = (connected) => states.push(connected);

    await transport.connect();
    gatt.disconnect();
    await vi.advanceTimersByTimeAsync(1500);

    expect(gatt.connect).toHaveBeenCalledTimes(2);
    expect(states).toEqual([false, true, false, true]);
    transport.disconnect();
  });

  it('no reintenta una desconexión manual', async () => {
    vi.useFakeTimers();
    const { gatt } = mockBluetooth(),
      transport = new BleTransport();

    await transport.connect();
    transport.disconnect();
    await vi.advanceTimersByTimeAsync(1500);

    expect(gatt.connect).toHaveBeenCalledTimes(1);
  });
});
