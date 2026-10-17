import type { RobotSample } from '../domain/types';
export function Sensors({ last }: { last: RobotSample | null }) {
  return (
    <section className="panel">
      <h2>Sensores y movimiento</h2>
      <div className="state">{last?.state ?? 'Sin datos'}</div>
      <div className="tiles">
        {(['I', 'F', 'D'] as const).map((key, i) => (
          <div className="tile" key={key} style={{ color: `var(--${['left', 'front', 'right'][i]})` }}>
            <div>{['Izquierdo (L)', 'Frontal (F)', 'Derecho (D)'][i]}</div>
            <strong>{last ? (last[key] < 0 ? 'Sin lectura' : `${last[key]} mm`) : '—'}</strong>
          </div>
        ))}
      </div>
      <svg viewBox="0 0 400 170" aria-label="Distancias vistas por el ratón">
        <rect x="175" y="100" width="50" height="60" rx="8" fill="var(--text)" />
        {(['I', 'F', 'D'] as const).map((key, i) => {
          const dist = last?.[key] ?? -1,
            len = Math.min(dist < 0 ? 140 : dist / 3, 140);
          return (
            <line
              key={key}
              x1={i === 0 ? 175 : i === 1 ? 200 : 225}
              y1={i === 1 ? 100 : 115}
              x2={i === 0 ? 175 - len : i === 1 ? 200 : 225 + len}
              y2={i === 1 ? 100 - len * 0.6 : 115}
              stroke={`var(--${['left', 'front', 'right'][i]})`}
              strokeWidth="3"
              strokeDasharray={dist < 0 ? '5 5' : undefined}
            />
          );
        })}
      </svg>
      <p>
        Velocidad: {last?.v ?? '—'} mm/s · Rumbo: {last?.r ?? '—'}° · Pasillo: {last?.p ?? '—'} mm
      </p>
      <p>
        Confianza L/F/D: {last?.confidence?.join(' / ') ?? '—'} % · Ciclo: {last?.cycle ?? '—'} ms
      </p>
    </section>
  );
}
export function Chart({ samples }: { samples: RobotSample[] }) {
  const end = samples.at(-1)?.t ?? 0;
  return (
    <section className="panel">
      <h2>Láseres · últimos 20 s (mm)</h2>
      <svg viewBox="0 0 600 260" role="img" aria-label="Gráfica temporal de sensores">
        {[0, 100, 200, 300, 400].map((v) => (
          <g key={v}>
            <line x1="35" x2="590" y1={230 - v * 0.5} y2={230 - v * 0.5} stroke="var(--grid)" />
            <text x="0" y={235 - v * 0.5} fill="var(--muted)" fontSize="12">
              {v}
            </text>
          </g>
        ))}
        {(['I', 'F', 'D'] as const).map((k, i) => {
          let d = '',
            pen = false;
          for (const s of samples) {
            if (s[k] < 0) {
              pen = false;
              continue;
            }
            d += `${pen ? 'L' : 'M'}${35 + 555 * (1 - (end - s.t) / 20000)},${230 - Math.min(400, s[k]) * 0.5} `;
            pen = true;
          }
          return (
            <path
              key={k}
              d={d}
              fill="none"
              stroke={`var(--${['left', 'front', 'right'][i]})`}
              strokeWidth="2"
            />
          );
        })}
        <text x="35" y="255" fill="var(--muted)" fontSize="12">
          −20 s
        </text>
        <text x="550" y="255" fill="var(--muted)" fontSize="12">
          ahora
        </text>
      </svg>
    </section>
  );
}
