import { useEffect, useMemo, useRef, useState } from 'react';
import type { Catalog } from '../telemetria/types';
import { fetchSession, loadCatalog } from './catalog';
import { useReplay } from './useReplay';
import { ReplayControls } from './ReplayControls';
import { SimulationMap } from '../mapa/Maps';
const formatAge = (ms: unknown) => (typeof ms === 'number' ? Math.round(ms).toString() : '—');

export function Simulation() {
  const [catalog, setCatalog] = useState<Catalog | null>(null),
    [error, setError] = useState(''),
    [controller, setController] = useState(''),
    [suite, setSuite] = useState('all'),
    [filter, setFilter] = useState(''),
    [selected, setSelected] = useState('');
  const replay = useReplay(),
    request = useRef(0);
  useEffect(() => {
    let active = true;
    const requests = request; // contador de cargas: al desmontar invalida las pendientes
    loadCatalog()
      .then((data) => {
        if (active) {
          setCatalog(data);
          setController(data.default_controller);
        }
      })
      .catch((e) => {
        if (active) setError(String(e));
      });
    return () => {
      active = false;
      requests.current++;
    };
  }, []);
  const rows = useMemo(
    () =>
      catalog?.results.filter(
        (r) =>
          r.controller === controller &&
          (suite === 'all' ||
            (suite === '21' && ['competition', 'altered', 'random'].includes(r.suite)) ||
            r.suite === suite),
      ) ?? [],
    [catalog, controller, suite],
  );
  if (!catalog)
    return (
      <section className="panel">
        <h2>Laboratorio de simulación</h2>
        <p role="status">{error || 'Cargando resultados…'}</p>
      </section>
    );
  const choose = async (file: string, id: string) => {
    const generation = ++request.current;
    try {
      const data = await fetchSession(file);
      if (generation !== request.current) return;
      replay.open(data);
      setSelected(id);
      setError('');
    } catch (e) {
      if (generation === request.current) setError(String(e));
    }
  };
  return (
    <>
      <section className="panel">
        <h2>Laboratorio de resultados</h2>
        <p>
          Actualizado: {new Date(catalog.generated_at).toLocaleString('es')} · {catalog.run_id}
        </p>
        <p>Solo cuenta como éxito llegar a la meta sin contactos. Una parada no es un éxito.</p>
        <div className="row">
          <label>
            Controlador{' '}
            <select value={controller} onChange={(e) => setController(e.target.value)}>
              {[...new Set(catalog.results.map((r) => r.controller))].map((c) => (
                <option key={c} value={c}>
                  {catalog.controller_labels[c] ?? c}
                </option>
              ))}
            </select>
          </label>
          <label>
            Banco{' '}
            <select value={suite} onChange={(e) => setSuite(e.target.value)}>
              <option value="all">Todos</option>
              <option value="21">Banco principal (21 pistas)</option>
              {[...new Set(catalog.results.map((r) => r.suite))].map((s) => (
                <option key={s} value={s}>
                  {(
                    {
                      competition: 'Competencia',
                      altered: 'Pared cambiada',
                      random: 'Pistas aleatorias',
                      extremes: 'Casos extremos',
                    } as Record<string, string>
                  )[s] ?? s}
                </option>
              ))}
            </select>
          </label>
        </div>
        <div className="tiles metrics">
          {[
            [rows.filter((r) => r.clean_arrival).length + '/' + rows.length, 'Llegadas limpias'],
            [rows.filter((r) => !r.clean_arrival).length, 'Fallos'],
            [rows.reduce((a, r) => a + r.contacts, 0), 'Contactos'],
            [rows.length, 'Pruebas'],
          ].map(([v, label]) => (
            <div className="tile" key={label}>
              <strong>{v}</strong>
              <div>{label}</div>
            </div>
          ))}
        </div>
      </section>
      <div className="grid">
        <section className="panel">
          <h2>Sesiones disponibles</h2>
          {catalog.replays.map((item) => (
            <button
              className="session"
              aria-pressed={selected === item.id}
              key={item.id}
              onClick={() => void choose(item.file, item.id)}
            >
              {item.label}
              <small>{item.summary}</small>
            </button>
          ))}
        </section>
        <section className="panel">
          <h2>Reproducción de simulación</h2>
          {error && <p role="alert">{error}</p>}
          {replay.session ? (
            <>
              <SimulationMap data={replay.session.data} time={replay.session.time} />
              <ReplayControls replay={replay} />
              <div className="tiles">
                {(['L', 'F', 'R'] as const).map((key, i) => (
                  <div className="tile" key={key}>
                    {['Láser izquierdo', 'Láser frontal', 'Láser derecho'][i]}
                    <div>
                      {replay.session!.sample.filtered[key] < 0
                        ? 'Sin lectura'
                        : Math.round(replay.session!.sample.filtered[key]) + ' mm'}
                    </div>
                    <small>
                      Edad: {formatAge(replay.session!.sample.sensors?.[i]?.[3])} ms · Confianza:{' '}
                      {replay.session!.sample.sensors
                        ? Math.round(Number(replay.session!.sample.sensors[i][2]) * 100) + '%'
                        : '—'}
                    </small>
                  </div>
                ))}
              </div>
              <a href={`?page=telemetry&replay=${encodeURIComponent(selected)}`}>Abrir en Telemetría</a>
            </>
          ) : (
            <p>Selecciona una sesión para revisar el recorrido.</p>
          )}
        </section>
      </div>
      <section className="panel">
        <h2>Resultados completos</h2>
        <label>
          Filtrar por modo, mapa, ancho o resultado{' '}
          <input type="search" value={filter} onChange={(e) => setFilter(e.target.value)} />
        </label>
        <div className="table-wrap">
          <table>
            <thead>
              <tr>
                {['Caso', 'Modo', 'Ancho', 'Resultado', 'Contactos', 'Tiempo'].map((s) => (
                  <th key={s}>{s}</th>
                ))}
              </tr>
            </thead>
            <tbody>
              {rows.map((r, i) => {
                const values = [
                  `${r.suite} · mapa ${r.maze} · semilla ${r.noise_seed}`,
                  r.mode,
                  r.corridor_mm === undefined ? '—' : `${r.corridor_mm} mm`,
                  [
                    r.clean_arrival
                      ? 'Llegó sin contacto'
                      : r.reached !== null
                        ? 'Llegó con contacto'
                        : 'No llegó',
                    r.failure,
                  ]
                    .filter(Boolean)
                    .join(' · '),
                  r.contacts,
                  (r.reached ?? r.elapsed).toFixed(1) + ' s',
                ];
                return values.join(' ').toLowerCase().includes(filter.toLowerCase()) ? (
                  <tr key={i}>
                    {values.map((v, j) => (
                      <td key={j}>{v}</td>
                    ))}
                  </tr>
                ) : null;
              })}
            </tbody>
          </table>
        </div>
      </section>
      <p className="status">
        Simulación 2D con geometría conservada. Los resultados no certifican el comportamiento físico.
      </p>
    </>
  );
}
