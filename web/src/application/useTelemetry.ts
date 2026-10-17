import { useCallback, useEffect, useReducer, useRef, useState } from 'react';
import { BleTransport } from '../infrastructure/BleTransport';
import { CommandQueue } from '../infrastructure/CommandQueue';
import { initialState, telemetryReducer } from './state';
import { initialSupervisor, supervise } from '../domain/supervisor';
import { parseRobot } from '../domain/robot';

export function useTelemetry() {
  const [state, dispatch] = useReducer(telemetryReducer, undefined, initialState);
  const [connected, setConnected] = useState(false);
  const [name, setName] = useState('');
  const [error, setError] = useState('');
  const [tick, setTick] = useState(0);
  const [transport] = useState(() => new BleTransport());
  const [queue] = useState(() => new CommandQueue(transport));
  const replay = useRef(false);
  const start = useRef(performance.now());
  const settings = useRef({ mode: 1, supervisor: false, learn: true, speed: 100, range: 300 });
  const supervisor = useRef(initialSupervisor());

  useEffect(() => {
    transport.onConnection = (on, deviceName) => {
      setConnected(on);
      setName(deviceName);
      if (!on) queue.cancel();
    };
    transport.onError = setError;
    transport.onLine = (line) => {
      if (replay.current) return;
      if (queue.acknowledge(line)) {
        setTick((t) => t + 1);
        return;
      }
      const t = performance.now() - start.current;
      dispatch({ type: 'line', line, t, range: settings.current.range });
      const sample = parseRobot(line, t);
      if (sample?.hasPose) {
        const config = settings.current;
        const result = supervise(
          supervisor.current,
          sample,
          config.mode,
          config.supervisor,
          config.learn,
          config.speed,
        );
        supervisor.current = result.state;
        for (const cmd of result.commands) void queue.send(cmd);
      }
      setTick((n) => n + 1);
    };
    return () => {
      queue.cancel();
      transport.disconnect();
    };
  }, [queue, transport]);

  const send = useCallback(
    async (cmd: string) => {
      if (replay.current || !transport.connected) return false;
      const ok = await queue.send(cmd);
      dispatch({
        type: 'line',
        line: `Orden ${ok ? 'confirmada' : 'sin confirmar'}: ${cmd}`,
        t: performance.now() - start.current,
      });
      return ok;
    },
    [queue, transport],
  );

  const connect = async () => {
    try {
      setError('');
      await transport.connect();
    } catch (e) {
      setError(e instanceof Error ? e.message : String(e));
    }
  };
  const reset = () => {
    dispatch({ type: 'reset' });
    supervisor.current = initialSupervisor(settings.current.speed);
    start.current = performance.now();
  };
  return {
    state,
    dispatch,
    connected,
    name,
    error,
    setError,
    connect,
    transport,
    queue,
    send,
    replay,
    settings,
    supervisor,
    reset,
    tick,
    startedAt: start,
  };
}
