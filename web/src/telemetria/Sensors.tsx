import type { RobotSample } from './types';
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
      <Radar last={last} />
      <p>
        Velocidad: {last?.v ?? '—'} mm/s · Rumbo: {last?.r ?? '—'}° · Pasillo: {last?.p ?? '—'} mm
      </p>
      <p>
        Confianza L/F/D: {last?.confidence?.join(' / ') ?? '—'} % · Ciclo: {last?.cycle ?? '—'} ms
      </p>
    </section>
  );
}
// Lo que ve cada VL53L0X, como un radar: el frontal y los laterales montados a 45 grados iluminan un cono
// de 25 grados; la lectura es la distancia (media) a lo que hay dentro del cono. Vista desde arriba con el
// ratón mirando hacia arriba; las líneas discontinuas son las paredes de un pasillo centrado.
const SCALE = 0.5; // px por mm
const HALF_FOV = 12.5;
const BEAMS = [
  { key: 'I', ahead: 30, left: 30, angle: 45, color: 'left' },
  { key: 'F', ahead: 40, left: 0, angle: 0, color: 'front' },
  { key: 'D', ahead: 30, left: -30, angle: -45, color: 'right' },
] as const;
export function Radar({ last }: { last: RobotSample | null }) {
  const cx = 200,
    cy = 235,
    toScreen = (ahead: number, left: number) => [cx - left * SCALE, cy - ahead * SCALE],
    dir = (deg: number) => [-Math.sin((deg * Math.PI) / 180), -Math.cos((deg * Math.PI) / 180)];
  const corridor = last && last.p > 50 ? last.p : 143;
  return (
    <svg viewBox="0 0 400 250" role="img" aria-label="Conos de los tres sensores, vista de radar">
      {[-1, 1].map((side) => (
        <line
          key={side}
          x1={cx + side * (corridor / 2) * SCALE}
          x2={cx + side * (corridor / 2) * SCALE}
          y1={0}
          y2={250}
          stroke="var(--grid)"
          strokeDasharray="4 4"
        />
      ))}
      {BEAMS.map((b) => {
        const dist = last?.[b.key] ?? -1,
          r = (dist < 0 ? 600 : Math.min(dist, 600)) * SCALE,
          [ox, oy] = toScreen(b.ahead, b.left),
          [ax, ay] = dir(b.angle + HALF_FOV),
          [bx, by] = dir(b.angle - HALF_FOV),
          [mx, my] = dir(b.angle);
        return (
          <g key={b.key}>
            <path
              d={`M${ox},${oy} L${ox + ax * r},${oy + ay * r} A${r},${r} 0 0 1 ${ox + bx * r},${oy + by * r} Z`}
              fill={`var(--${b.color})`}
              fillOpacity={dist < 0 ? 0.05 : 0.18}
              stroke={`var(--${b.color})`}
              strokeDasharray={dist < 0 ? '5 5' : undefined}
            />
            <text
              x={ox + mx * Math.min(r, 120) + (b.angle > 0 ? -40 : b.angle < 0 ? 4 : -14)}
              y={oy + my * Math.min(r, 120) - 4}
              fill={`var(--${b.color})`}
              fontSize="13"
            >
              {dist < 0 ? '—' : `${dist}`}
            </text>
          </g>
        );
      })}
      <circle cx={cx} cy={cy} r={45 * SCALE} fill="var(--text)" />
      <path d={`M${cx},${cy - 18} l-6,10 l12,0 Z`} fill="var(--bg, white)" />
    </svg>
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
