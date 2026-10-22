import { lazy, Suspense, useEffect, useMemo, useState } from 'react';
import { useTelemetry } from './useTelemetry';
import { useRobotCommands, modes, modeHelp } from '../conexion/useRobotCommands';
import { Sensors, Chart } from './Sensors';
import { Diagnostics } from '../seguridad/Diagnostics';
import { Parameters } from '../parametros/Parameters';
import { downloadRecording } from './recording';
import { LiveMap2D } from '../mapa/LiveMap';
import { liveMaze } from '../mapa/liveMaze';
import { spatialPose } from '../mapa/spatialPose';
const Map3D = lazy(() => import('../mapa/Map3D'));
export function Telemetry() {
  const live = useTelemetry();
  const commands = useRobotCommands(live.send);
  const [now, setNow] = useState(0);
  const [busy, setBusy] = useState(false);
  const [view, setView] = useState<'2d' | '3d'>('2d');
  useEffect(() => {
    const timer = setInterval(() => setNow(performance.now()), 200);
    return () => clearInterval(timer);
  }, []);
  const { state } = live;
  const clock = now - live.startedAt.current;
  const sample = state.last;
  const compatible = state.firmwareV3;
  const canSend =
    live.connected &&
    compatible &&
    !busy &&
    !state.resetPending &&
    !state.awaitingResetBoot &&
    !state.calibrating;
  const resetSelection = commands.resetSelection;
  useEffect(() => {
    if (state.resetRevision > 0) resetSelection();
  }, [state.resetRevision, resetSelection]);
  const maze = useMemo(
    () => liveMaze(state.boardParams, state.cells, sample?.pitch),
    [state.boardParams, state.cells, sample?.pitch],
  );
  const elapsed = state.raceStart === null ? 0 : ((state.raceEnd ?? clock) - state.raceStart) / 1000;
  const stay = 0;
  const run = async (fn: () => Promise<unknown>) => {
    setBusy(true);
    try {
      await fn();
    } catch (e) {
      live.setError(String(e));
    } finally {
      setBusy(false);
    }
  };
  return (
    <>
      <section className="panel connection-panel">
        <div className="row">
          <strong role="status">
            {live.connected ? 'Conectado a ' + live.name : 'Esperando al robot autorizado'}
          </strong>
          <button disabled={busy} onClick={() => void run(live.connect)}>
            Vincular robot
          </button>
          <button disabled={!live.connected} onClick={() => live.transport.disconnect()}>
            Desconectar
          </button>
          <button className="go" disabled={!canSend} onClick={() => void run(commands.start)}>
            INICIAR
          </button>
          <button className="stop" disabled={!live.connected} onClick={() => void commands.stop()}>
            PARAR
          </button>
          <button
            className="stop"
            disabled={!live.connected || !compatible || busy || state.resetPending || !state.resetAvailable}
            onClick={() =>
              void run(async () => {
                live.setError('');
                if (!(await live.resetAll()))
                  live.setError('No se confirmó la solicitud de reinicio. Comprueba la conexión.');
              })
            }
          >
            {state.resetPending ? 'REINICIANDO…' : 'REINICIAR TODO'}
          </button>
        </div>
        <button
          className="go"
          disabled={!canSend || !state.calibrationAvailable}
          onClick={() => void run(() => live.send('calall'))}
        >
          {state.calibrating ? 'CALIBRANDO…' : 'CALIBRACIÓN COMPLETA'}
        </button>
        <p className="status">
          Coloca el robot centrado en la salida de un pasillo de ancho conocido, con paredes a ambos lados,
          una celda libre delante y una pared frontal visible a menos de 60 cm. Retira las manos: girará,
          avanzará lentamente y comprobará el regreso antes de guardar.
        </p>
        <p className="status">
          Reiniciar todo detiene el robot, restaura ajustes y calibraciones de fábrica y borra los mapas
          guardados y la trayectoria.
        </p>
        <p className="status">
          Vincula “Raton” una vez. Después se intentará reconectar automáticamente; reconectar no inicia ni
          reinicia una misión.
        </p>
        {live.connected && !compatible && (
          <p role="alert">Esperando firmware autonomia-v3. Los modos se habilitan al confirmar la versión.</p>
        )}
        {live.error && (
          <p role="alert" className="danger">
            {live.error}
          </p>
        )}
        <div className="modes" aria-label="Modos del robot">
          {modes.map((name, i) => (
            <button
              key={name}
              className={'mode ' + (commands.mode === i + 1 ? 'sel' : '')}
              aria-pressed={commands.mode === i + 1}
              disabled={!canSend}
              onClick={() => commands.select(i + 1)}
            >
              <b>
                {i + 1}. {name}
              </b>
              <span>{modeHelp[i]}</span>
            </button>
          ))}
        </div>
        <p>
          <strong>{state.decision}</strong>
        </p>
        {state.syncing && <p role="status">Recuperando estado, mapa y eventos del robot…</p>}
      </section>
      <div className="grid">
        <section className="panel wide">
          <div className="row">
            <h2>Mapa y trayectoria estimada</h2>
            <button aria-pressed={view === '2d'} onClick={() => setView('2d')}>
              2D
            </button>
            <button aria-pressed={view === '3d'} onClick={() => setView('3d')}>
              3D
            </button>
            <button disabled={!canSend} onClick={() => void run(commands.loadMap)}>
              Leer mapa guardado
            </button>
            <button onClick={() => downloadRecording(state.record)}>Guardar registro</button>
          </div>
          <p className="status">
            Paredes confirmadas por la placa y posición estimada con sensores, IMU y encoders. Las zonas sin
            observaciones siguen desconocidas.
          </p>
          {sample?.hasPose && !spatialPose(sample, maze) && (
            <p role="alert" className="danger">
              Posición estimada fuera de la pista: x={sample.x} mm, y={sample.y} mm. Se conserva en el
              registro; no se dibuja como recorrido válido.
            </p>
          )}
          {view === '2d' ? (
            <LiveMap2D maze={maze} cells={state.cells} trail={state.trail} last={sample} />
          ) : (
            <Suspense fallback={<p>Cargando vista 3D…</p>}>
              <Map3D occupancy={state.occupancy} trail={state.trail} last={sample} maze={maze} />
            </Suspense>
          )}
        </section>
        <Sensors
          last={sample}
          readings={state.sensorReadings}
          now={clock}
          sideControl={state.sideControl}
          geometry={state.boardParams}
        />
        <Chart frames={state.sensorHistory} now={clock} />
        <Diagnostics state={state} elapsed={elapsed} stay={stay} />
      </div>
      <details className="panel">
        <summary>Ajustes y diagnóstico detallado</summary>
        <p>La autonomía se ejecuta en la placa y continúa sin la web.</p>
        <div className="row">
          <button disabled={!canSend} onClick={() => void run(() => live.send('sensores'))}>
            Lecturas sin mover
          </button>
          <button disabled={!canSend} onClick={() => void run(() => live.send('pruebasensores'))}>
            Autoprueba de cinco sensores
          </button>
        </div>
        <Parameters
          key={state.resetRevision}
          disabled={!canSend}
          sequence={commands.sequence}
          boardParams={state.boardParams}
          send={live.send}
          onApplied={(speed, range) => {
            live.settings.current = { ...live.settings.current, speed, range };
          }}
        />
        <h2>Registro de decisiones</h2>
        <pre className="event-log">{state.events.join('\n') || 'Esperando datos del robot.'}</pre>
      </details>
    </>
  );
}
