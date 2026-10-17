import type { TelemetryState } from '../application/state';

export function Diagnostics({
  state,
  elapsed,
  stay,
}: {
  state: TelemetryState;
  elapsed: number;
  stay: number;
}) {
  return (
    <section className="panel wide">
      <h2>Decisiones y diagnóstico</h2>
      <p>
        <strong>{state.decision}</strong>
      </p>
      <p>
        Tiempo: {elapsed.toFixed(1)} s / 600 · En la misma zona: {stay.toFixed(1)} s / 30 ·{' '}
        {state.raceEnd !== null
          ? '¡En la meta!'
          : elapsed > 600
            ? 'Tiempo máximo superado'
            : stay > 30
              ? 'Más de 30 s sin salir de la zona'
              : 'Esperando / en carrera'}
      </p>
      <div className="tiles">
        {Object.entries(state.diagnostics).map(([kind, fields]) => (
          <div className="tile" key={kind}>
            <strong>{kind === 'OBS' ? 'Observaciones' : 'Maniobra'}</strong>
            {Object.entries(fields)
              .filter(([k]) => k !== 'kind')
              .map(([key, value]) => (
                <div key={key}>
                  {(
                    {
                      raw: 'Lecturas crudas L/F/D',
                      age: 'Edad (ms)',
                      conf: 'Confianza',
                      enc: 'Encoders',
                      phase: 'Fase',
                      drops: 'Líneas descartadas',
                      completed: 'Completada',
                      reason: 'Motivo',
                      actual: 'Avance (mm)',
                      error_cdeg: 'Error angular (centigrados)',
                    } as Record<string, string>
                  )[key] ?? key}
                  : {Array.isArray(value) ? value.join(' · ') : value}
                </div>
              ))}
          </div>
        ))}
      </div>
    </section>
  );
}
