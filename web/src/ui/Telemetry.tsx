import { Diagnostics } from './Diagnostics';
import { lazy, Suspense, useEffect, useMemo, useRef, useState } from 'react';
import { useTelemetry } from '../application/useTelemetry';
import { useRobotCommands, modes } from '../application/useRobotCommands';
import { useReplay } from '../application/useReplay';
import { ReplayProjector } from '../application/replayState';
import { competition } from '../domain/competition';
import { mazeFromReplay, poseCell, worldPose } from '../domain/maze';
import type { Catalog, Cell, ReplayData } from '../domain/types';
import { fetchSession, loadCatalog } from '../infrastructure/catalog';
import { downloadRecording, loadRecording } from '../infrastructure/recording';
import { Track, LearnedMap } from './Maps';
import { Sensors, Chart } from './Sensors';
import { Parameters } from './Parameters';
import { ReplayControls } from './ReplayControls';

const Map3D = lazy(() => import('./Map3D'));
export function Telemetry() {
  const live = useTelemetry(),
    commands = useRobotCommands(live.send),
    replay = useReplay(),
    [catalog, setCatalog] = useState<Catalog | null>(null),
    [example, setExample] = useState(''),
    [label, setLabel] = useState(''),
    [learnGoal, setLearnGoal] = useState<Cell | null>(null),
    [trackGoal, setTrackGoal] = useState<Cell>(competition.goal),
    [sup, setSup] = useState(false),
    [learn, setLearn] = useState(true),
    [now, setNow] = useState(0),
    [busy, setBusy] = useState(false);
  const request = useRef(0);
  const active = !!replay.session;
  live.replay.current = active;
  live.settings.current = { ...live.settings.current, mode: commands.mode, supervisor: sup, learn };
  const replayData = replay.session?.data,
    replayTime = replay.session?.time ?? 0;
  const projector = useMemo(() => (replayData ? new ReplayProjector(replayData) : null), [replayData]);
  const replayState = useMemo(() => projector?.at(replayTime) ?? null, [projector, replayTime]);
  const state = replayState ?? live.state,
    maze = useMemo(
      () => (replayData ? mazeFromReplay(replayData, competition) : { ...competition, goal: trackGoal }),
      [replayData, trackGoal],
    );
  const canSend = live.connected && !active;
  const open = (data: ReplayData, name: string) => {
    if (live.transport.connected) throw Error('Desconecta el robot antes de abrir una reproducción.');
    live.queue.cancel();
    live.replay.current = true;
    replay.open(data);
    setLabel(name);
    live.setError('');
  };
  const openRef = useRef(open);
  openRef.current = open;
  const setError = live.setError;
  useEffect(() => {
    let mounted = true;
    const requests = request; // contador de cargas: al desmontar invalida las pendientes
    loadCatalog()
      .then(async (data) => {
        if (!mounted) return;
        setCatalog(data);
        setExample(data.replays[0]?.id ?? '');
        const id = new URLSearchParams(location.search).get('replay'),
          item = data.replays.find((r) => r.id === id);
        if (item) {
          const session = await fetchSession(item.file);
          if (mounted) openRef.current(session, item.label);
        }
      })
      .catch((e) => {
        if (mounted) setError(String(e));
      });
    const timer = setInterval(() => setNow(performance.now()), 250);
    return () => {
      mounted = false;
      requests.current++;
      clearInterval(timer);
    };
  }, [setError]);
  const loadExample = async () => {
    const id = ++request.current,
      item = catalog?.replays.find((r) => r.id === example);
    if (!item) return;
    try {
      const data = await fetchSession(item.file);
      if (id === request.current) open(data, item.label);
    } catch (e) {
      live.setError(String(e));
    }
  };
  const sample = state.last,
    cell = sample ? poseCell(worldPose(sample, maze), sample.a, maze.PITCH) : null;
  const stay = (() => {
    if (!sample || !cell) return 0;
    let from = sample.t;
    for (let i = state.trail.length - 1; i >= 0; i--) {
      const s = state.trail[i],
        c = poseCell(worldPose(s, maze), s.a, maze.PITCH);
      if (c[0] !== cell[0] || c[1] !== cell[1]) break;
      from = s.t;
    }
    return (sample.t - from) / 1000;
  })();
  const elapsed =
    state.raceStart !== null
      ? ((state.raceEnd ?? (active ? replayTime * 1000 : now - live.startedAt.current)) - state.raceStart) /
        1000
      : active
        ? replayTime
        : 0;
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
      <section className="panel">
        <div className="row">
          <strong>
            {active
              ? 'Reproducción · ' + label
              : live.connected
                ? 'Conectado a ' + live.name
                : 'Sin conexión'}
          </strong>
          <button disabled={active || busy} onClick={() => void run(live.connect)}>
            Conectar
          </button>
          <button disabled={!live.connected} onClick={() => live.transport.disconnect()}>
            Desconectar
          </button>
          <button className="go" disabled={!canSend || busy} onClick={() => void run(commands.start)}>
            ARRANCAR
          </button>
          <button className="stop" disabled={!canSend} onClick={() => void commands.stop()}>
            PARAR
          </button>
          <button
            disabled={!canSend || busy}
            onClick={() => {
              if (confirm('¿Reiniciar la placa y borrar el mapa aprendido?')) void run(commands.restart);
            }}
          >
            REINICIAR
          </button>
        </div>
        <p role="status">
          Placa: {state.boardMode} · Enlace: {live.transport.stats.ok} válidas · {live.transport.stats.bad}{' '}
          corruptas · {live.transport.stats.lost} perdidas · {live.transport.stats.duplicates} duplicadas ·{' '}
          {live.queue.acked} ACK · {live.queue.retries} reintentos
        </p>
        {!navigator.bluetooth && (
          <p>Web Bluetooth requiere Chrome o Edge en un dispositivo compatible y HTTPS o localhost.</p>
        )}
        {live.error && (
          <p role="alert" className="danger">
            {live.error}
          </p>
        )}
        <div className="row">
          <label>
            Sesión de simulación{' '}
            <select disabled={live.connected} value={example} onChange={(e) => setExample(e.target.value)}>
              {catalog?.replays.map((item) => (
                <option key={item.id} value={item.id}>
                  {item.label} · {item.summary}
                </option>
              ))}
            </select>
          </label>
          <button disabled={live.connected || !example} onClick={() => void loadExample()}>
            Abrir sesión
          </button>
          <label>
            Importar registro .txt / .json{' '}
            <input
              type="file"
              accept=".txt,.json"
              disabled={live.connected}
              onChange={(e) => {
                const file = e.target.files?.[0];
                if (file)
                  void loadRecording(file)
                    .then((data) => open(data, file.name))
                    .catch((e) => live.setError(String(e)));
                e.target.value = '';
              }}
            />
          </label>
          {active && (
            <button
              onClick={() => {
                request.current++;
                live.replay.current = false;
                replay.close();
                live.reset();
              }}
            >
              Volver a conexión
            </button>
          )}
        </div>
        <ReplayControls replay={replay} />
        <p className="status">La reproducción desactiva todas las órdenes al hardware y el supervisor.</p>
      </section>
      <div className="grid">
        <section className="panel wide">
          <h2>Modos</h2>
          <div className="modes">
            {modes.map((name, i) => (
              <button
                className={'mode ' + (commands.mode === i + 1 ? 'sel' : '')}
                aria-pressed={commands.mode === i + 1}
                disabled={active || busy}
                key={name}
                onClick={() => commands.select(i + 1)}
              >
                <b>
                  {i + 1} · {name}
                </b>
                <span>
                  {
                    [
                      'Sigue la pared derecha',
                      'Sigue la pared izquierda',
                      'Explora o recorre el mapa aprendido',
                      'Ruta mínima del reglamento',
                      'Elige el mayor espacio libre',
                      'Mapea, vuelve a la salida y guarda en flash',
                    ][i]
                  }
                </span>
              </button>
            ))}
          </div>
          <p className="status">
            Elegir modo envía m1–m6 si está conectado. ARRANCAR envía el modo seleccionado.
          </p>
          {(commands.mode === 3 || commands.mode === 6 || Object.keys(state.cells).length > 0) && (
            <>
              <h2>Mapa aprendido en vivo</h2>
              <div className="row">
                <button
                  disabled={!canSend || busy}
                  onClick={() => {
                    setLearnGoal(null);
                    void run(commands.train);
                  }}
                >
                  Aprender pista y guardar
                </button>
                <button
                  disabled={!canSend || busy || !learnGoal || !Object.keys(state.cells).length}
                  onClick={() => learnGoal && void run(() => commands.route(learnGoal))}
                >
                  Ruta más corta a la meta
                </button>
              </div>
              <button disabled={!canSend || busy || active} onClick={() => void run(commands.loadMap)}>
                Cargar mapa guardado
              </button>
              <button disabled={!canSend || busy || active} onClick={() => void run(commands.eraseMap)}>
                Borrar mapa guardado
              </button>
              <p className="status">
                MAPA_GUARDADO: disponible tras reiniciar. MAPA_CARGADO: listo para ruta con meta.
                MAPA_INCOMPLETO / MAPA_LIMITE: no se guarda. MAPA_ERROR_FLASH: fallo de memoria. Coloca el
                ratón en la salida original mirando al norte antes de recorrer el mapa.
              </p>
              <LearnedMap
                cells={state.cells}
                robot={state.learnRobot}
                goal={learnGoal}
                onGoal={setLearnGoal}
                disabled={active}
              />
            </>
          )}
        </section>
        <Sensors last={sample} />
        <Chart samples={state.samples} />
        <Diagnostics state={state} elapsed={elapsed} stay={stay} />
        <Track
          maze={maze}
          last={sample}
          trail={state.trail}
          replaying={active}
          onGoal={(c) => {
            setTrackGoal(c);
            setLearnGoal(c);
            if (canSend) void live.send(`G${c.join(',')}`);
          }}
        />
        <section className="panel wide">
          <div className="row">
            <button onClick={() => live.dispatch({ type: 'clearMap' })} disabled={active}>
              Borrar mapa recorrido
            </button>
            <button onClick={() => downloadRecording(state.record)}>Descargar registro .txt</button>
          </div>
        </section>
        <Suspense fallback={<section className="panel wide">Cargando mapa 3D…</section>}>
          <Map3D occupancy={state.occupancy} trail={state.trail} last={sample} maze={maze} />
        </Suspense>
        <Parameters
          disabled={!canSend || busy}
          sequence={commands.sequence}
          send={live.send}
          onApplied={(speed, range) => {
            live.settings.current = { ...live.settings.current, speed, range };
            live.supervisor.current = { ...live.supervisor.current, speed };
          }}
        />
        <section className="panel">
          <h2>Supervisor y aprendizaje de umbrales</h2>
          <label className="field">
            <input
              type="checkbox"
              checked={sup}
              disabled={active}
              onChange={(e) => setSup(e.target.checked)}
            />{' '}
            Corregir bucles, choques y atascos
          </label>
          <label className="field">
            <input
              type="checkbox"
              checked={learn}
              disabled={active}
              onChange={(e) => setLearn(e.target.checked)}
            />{' '}
            Aprender y enviar umbrales de la pista
          </label>
          <p>{active ? 'Desactivado durante la reproducción' : live.supervisor.current.message}</p>
          <p>
            Umbral lateral: {live.supervisor.current.wall ?? '—'} mm · Punto muerto:{' '}
            {live.supervisor.current.dead ?? '—'} mm
          </p>
          <p className="status">
            En modos 3 y 4 los giros repetidos solo generan un aviso. Ante contactos frecuentes reduce la
            velocidad y la recupera después de un tramo tranquilo.
          </p>
        </section>
        <section className="panel wide">
          <h2>Registro de eventos y decisiones</h2>
          <pre className="event-log" aria-live="polite">
            {state.events.join('\n') || 'Esperando mensajes del robot…'}
          </pre>
        </section>
      </div>
    </>
  );
}
