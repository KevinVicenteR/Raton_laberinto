import { useCallback, useEffect, useReducer, useRef, useState } from 'react';
import { BleTransport } from '../conexion/BleTransport';
import { CommandQueue } from '../conexion/CommandQueue';
import { initialState, telemetryReducer } from './state';
import { initialSupervisor, supervise } from '../seguridad/supervisor';
import { parseRobot } from '../telemetria/robot';
import { resetRobot } from '../conexion/resetRobot';

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
  const settings = useRef({ mode: 1, supervisor: false, learn: true, speed: 100, range: 600 });
  const supervisor = useRef(initialSupervisor());

  useEffect(() => {
    transport.onConnection = (on, deviceName) => {
      setConnected(on);
      setName(deviceName);
      if (!on) queue.cancel();
      else {
        setError('');
        void queue.send('sync');
      }
    };
    transport.onError = setError;
    transport.onLine = (line) => {
      if (replay.current) return;
      if (queue.acknowledge(line)) {
        setTick((t) => t + 1);
        return;
      }
      const t = performance.now() - start.current;
      if (line === 'REINICIO_TOTAL_OK') {
        queue.cancel();
        settings.current = { mode: 1, supervisor: false, learn: true, speed: 100, range: 600 };
        supervisor.current = initialSupervisor();
        localStorage.setItem('raton-mode', '1');
      }
      if (line.startsWith('CAL_COMPLETA_OK')) void queue.send('sync');
      if (line.startsWith('REINICIO_TOTAL_ERROR_FLASH'))
        setError(
          'No se pudo completar el borrado en la placa. El robot está detenido; vuelve a intentar Reiniciar todo.',
        );
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
    const restore = () => {
      if (!replay.current && document.visibilityState !== 'hidden') void transport.autoConnect();
    };
    restore();
    const timer = setInterval(restore, 15000);
    document.addEventListener('visibilitychange', restore);
    return () => {
      clearInterval(timer);
      document.removeEventListener('visibilitychange', restore);
      queue.cancel();
      transport.dispose();
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
    resetAll: () => resetRobot(send, () => queue.cancel()),
    replay,
    settings,
    supervisor,
    reset,
    tick,
    startedAt: start,
  };
}
