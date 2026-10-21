import type { TelemetryState } from '../telemetria/state';
export function Diagnostics({ state, elapsed }: { state: TelemetryState; elapsed: number; stay?: number }) {
  const pose = state.diagnostics.POSE;
  const sigma = Array.isArray(pose?.sigma) ? pose.sigma : null;
  return (
    <section className="panel wide">
      <h2>Posición y autocorrección</h2>
      <div className="tiles">
        <div className="tile">
          <strong>Posición estimada</strong>
          <p>{pose ? `X ${pose.x} · Y ${pose.y} mm` : 'Esperando referencias'}</p>
          <p>Incertidumbre: {sigma ? sigma.join(' / ') + ' mm' : 'sin datos'}</p>
        </div>
        <div className="tile">
          <strong>Avance real</strong>
          <p>
            {pose?.phase === 'AVANCE_CONTRASTADO'
              ? 'Contrastado con distancias'
              : pose?.phase === 'ATASCADO'
                ? 'Atasco detectado; recuperando'
                : 'Estimado por fusión de sensores'}
          </p>
          <p>
            Celda: {pose?.confirmed === 1 ? 'avance contrastado' : 'pendiente de contraste externo'} · Visitas
            estimadas: {pose?.visited ?? '—'}
          </p>
        </div>
        <div className="tile">
          <strong>Correcciones</strong>
          <p>Indicios de colisión: {pose?.collisions ?? 0}</p>
          <p>Oscilaciones detectadas: {pose?.oscillations ?? 0}</p>
        </div>
        <div className="tile">
          <strong>Misión</strong>
          <p>{state.decision}</p>
          <p>{elapsed.toFixed(1)} s / 600 s</p>
        </div>
      </div>
      <details>
        <summary>Información técnica</summary>
        {Object.entries(state.diagnostics).map(([kind, fields]) => (
          <p key={kind}>
            <strong>{kind}</strong>:{' '}
            {Object.entries(fields)
              .filter(([k]) => k !== 'kind')
              .map(([k, v]) => k + ': ' + (Array.isArray(v) ? v.join(' / ') : v))
              .join(' · ')}
          </p>
        ))}
        <p>
          Historial pendiente: últimos 32 eventos en RAM. Eventos antiguos sustituidos:{' '}
          {state.historyOverwritten}. El mapa completo se sincroniza aparte.
        </p>
      </details>
    </section>
  );
}
