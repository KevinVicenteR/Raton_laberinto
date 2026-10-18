import { useEffect, useState } from 'react';
import { Simulation } from '../simulacion/Simulation';
import { Telemetry } from '../telemetria/Telemetry';

export function App() {
  const [page, setPage] = useState(
      new URLSearchParams(location.search).get('page') === 'simulation' ? 'simulation' : 'telemetry',
    ),
    [theme, setTheme] = useState(localStorage.getItem('raton-theme') ?? 'auto');
  useEffect(() => {
    if (theme === 'auto') delete document.documentElement.dataset.theme;
    else document.documentElement.dataset.theme = theme;
    localStorage.setItem('raton-theme', theme);
  }, [theme]);
  return (
    <main>
      <header>
        <h1>Ratón · {page === 'telemetry' ? 'Telemetría' : 'Simulación'}</h1>
        <nav aria-label="Páginas">
          {[
            ['telemetry', 'Telemetría'],
            ['simulation', 'Simulación'],
          ].map(([id, label]) => (
            <button
              key={id}
              aria-current={page === id ? 'page' : undefined}
              onClick={() => {
                setPage(id);
                history.replaceState(null, '', `?page=${id}`);
              }}
            >
              {label}
            </button>
          ))}
        </nav>
        <select aria-label="Tema" value={theme} onChange={(e) => setTheme(e.target.value)}>
          <option value="auto">Tema del sistema</option>
          <option value="light">Claro</option>
          <option value="dark">Oscuro</option>
        </select>
      </header>
      <div hidden={page !== 'telemetry'}>
        <Telemetry />
      </div>
      {page === 'simulation' && <Simulation />}
    </main>
  );
}
