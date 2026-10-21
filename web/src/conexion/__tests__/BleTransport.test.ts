import { afterEach, describe, expect, it, vi } from 'vitest';
import { BleTransport } from '../BleTransport';

function mockBluetooth(failFirstConnection = false) {
  const rx = { writeValue: vi.fn(async () => {}) },
    tx = { addEventListener: vi.fn(), removeEventListener: vi.fn(), startNotifications: vi.fn(async () => {}) },
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
  return { device, gatt };
}

afterEach(() => {
  vi.useRealTimers();
  vi.unstubAllGlobals();
});

describe('BleTransport', () => {
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
