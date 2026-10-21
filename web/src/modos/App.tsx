import { useEffect, useState } from 'react';
import { Telemetry } from '../telemetria/Telemetry';
export function App() {
  const [theme, setTheme] = useState(localStorage.getItem('raton-theme') ?? 'auto');
  useEffect(() => {
    if (theme === 'auto') delete document.documentElement.dataset.theme;
    else document.documentElement.dataset.theme = theme;
    localStorage.setItem('raton-theme', theme);
  }, [theme]);
  return (
    <main>
      <header>
        <div>
          <h1>Ratón autónomo</h1>
          <p>Control y mapa en vivo</p>
        </div>
        <select aria-label="Tema" value={theme} onChange={(e) => setTheme(e.target.value)}>
          <option value="auto">Tema del sistema</option>
          <option value="light">Claro</option>
          <option value="dark">Oscuro</option>
        </select>
      </header>
      <Telemetry />
    </main>
  );
}
