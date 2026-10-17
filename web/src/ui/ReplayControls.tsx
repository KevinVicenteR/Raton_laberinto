import type { useReplay } from '../application/useReplay';
export function ReplayControls({ replay }: { replay: ReturnType<typeof useReplay> }) {
  const { session } = replay;
  if (!session) return null;
  const data = session.data,
    hits = data.events?.filter((e) => e[0] <= session.time && e[1] === 'choque').length,
    ended = session.time >= session.duration;
  return (
    <>
      <div className="row">
        <button onClick={replay.toggle}>{session.playing ? 'Pausar' : 'Reproducir'}</button>
        <button onClick={() => replay.seek(0)}>Volver al inicio</button>
        <label>
          Velocidad{' '}
          <select value={session.speed} onChange={(e) => replay.speed(+e.target.value)}>
            {[0.5, 1, 2, 4, 8, 16].map((n) => (
              <option key={n} value={n}>
                {n}×
              </option>
            ))}
          </select>
        </label>
        <output>
          {session.time.toFixed(1)} / {session.duration.toFixed(1)} s
        </output>
      </div>
      <input
        aria-label="Tiempo de la sesión"
        type="range"
        min="0"
        max={session.duration}
        step="any"
        value={session.time}
        onChange={(e) => replay.seek(+e.target.value)}
        style={{ width: '100%' }}
      />
      <p>
        {hits === undefined
          ? 'Registro importado: llegada y contactos sin certificar'
          : `${hits} contactos${ended ? ' · ' + (data.clean_arrival ? 'Llegó sin contacto' : data.reached != null ? 'Llegó con contacto' : (data.failure ?? 'No llegó')) : ''}`}
      </p>
      <div className="row">
        {data.events
          ?.filter((e) => e[1] === 'choque')
          .map((e, i) => (
            <button key={i} onClick={() => replay.seek(e[0])}>
              × {e[0].toFixed(1)} s
            </button>
          ))}
      </div>
    </>
  );
}
