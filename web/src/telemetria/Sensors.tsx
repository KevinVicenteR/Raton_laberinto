import type { RobotSample, SensorReadings, SensorFrame } from './types';
import { currentReading, SENSOR_DEFS, sensorPath } from './sensorData';

export function Sensors({
  last,
  readings,
  now,
  sideControl,
}: {
  last: RobotSample | null;
  readings: SensorReadings;
  now: number;
  sideControl?: boolean;
}) {
  return (
    <section className="panel">
      <h2>Sensores y movimiento</h2>
      <div className="state">
        {last?.state ?? (Object.keys(readings).length ? 'Lecturas de sensores' : 'Sin datos')}
      </div>
      <div className="tiles sensor-tiles">
        {SENSOR_DEFS.map((sensor) => {
          const reading = currentReading(readings, sensor.key, now);
          const label = !reading
            ? readings[sensor.key]
              ? 'Sin datos recientes'
              : 'Sin datos'
            : reading.mm < 0
              ? 'Sin lectura'
              : `${reading.mm} mm`;
          return (
            <div className="tile" key={sensor.key} style={{ color: `var(--${sensor.color})` }}>
              <div className="sensor-label">{sensor.label}</div>
              <strong>{label}</strong>
            </div>
          );
        })}
      </div>
      <p className="status">
        Centrado con laterales:{' '}
        {sideControl === undefined ? 'sin confirmar' : sideControl ? 'activado' : 'desactivado'}. Las lecturas
        se muestran aunque el centrado esté desactivado.
      </p>
      <Radar last={last} readings={readings} now={now} />
      <p className="status">Vista esquemática: frontal, dos diagonales a 45° y dos laterales a 90°.</p>
      <p>
        Velocidad: {last?.v ?? '—'} mm/s · Rumbo: {last?.r ?? '—'}° · Pasillo: {last?.p ?? '—'} mm
      </p>
      <p>
        Confianza DI/F/DD: {last?.confidence?.join(' / ') ?? '—'} % · Ciclo: {last?.cycle ?? '—'} ms
      </p>
    </section>
  );
}

const SCALE = 0.5;
const HALF_FOV = 12.5;
export function Radar({
  last,
  readings,
  now,
}: {
  last: RobotSample | null;
  readings: SensorReadings;
  now: number;
}) {
  const cx = 200,
    cy = 210;
  const toScreen = (ahead: number, left: number) => [cx - left * SCALE, cy - ahead * SCALE];
  const dir = (deg: number) => [-Math.sin((deg * Math.PI) / 180), -Math.cos((deg * Math.PI) / 180)];
  const corridor = last && last.p > 50 ? last.p : 143;
  return (
    <svg
      className="sensor-radar"
      viewBox="0 0 400 300"
      role="img"
      aria-label="Radar de cinco sensores: frontal, diagonales y laterales"
    >
      <title>Vista superior del robot con cinco sensores</title>
      {[-1, 1].map((side) => (
        <line
          key={side}
          x1={cx + ((side * corridor) / 2) * SCALE}
          x2={cx + ((side * corridor) / 2) * SCALE}
          y1={0}
          y2={300}
          stroke="var(--grid)"
          strokeDasharray="4 4"
        />
      ))}
      {SENSOR_DEFS.map((b) => {
        const reading = currentReading(readings, b.key, now);
        const dist = reading?.mm ?? -1;
        const r = (dist < 0 ? 180 : Math.max(1, Math.min(dist, 360))) * SCALE;
        const [ox, oy] = toScreen(b.ahead, b.left);
        const [ax, ay] = dir(b.angle + HALF_FOV),
          [bx, by] = dir(b.angle - HALF_FOV),
          [mx, my] = dir(b.angle);
        return (
          <g key={b.key}>
            <path
              d={`M${ox},${oy} L${ox + ax * r},${oy + ay * r} A${r},${r} 0 0 1 ${ox + bx * r},${oy + by * r} Z`}
              fill={`var(--${b.color})`}
              fillOpacity={dist < 0 ? 0.04 : 0.18}
              stroke={`var(--${b.color})`}
              strokeDasharray={dist < 0 ? '5 5' : undefined}
            />
            <text
              x={ox + mx * Math.max(Math.abs(b.angle) === 90 ? 65 : 30, Math.min(r, 110))}
              y={oy + my * Math.min(r, 110) - 8}
              textAnchor="middle"
              fill={`var(--${b.color})`}
              fontSize="11"
            >
              {b.short} {dist < 0 ? '—' : `${dist} mm`}
            </text>
          </g>
        );
      })}
      <circle cx={cx} cy={cy} r={45 * SCALE} fill="var(--text)" />
      <path d={`M${cx},${cy - 18} l-6,10 l12,0 Z`} fill="var(--bg, white)" />
    </svg>
  );
}

export function Chart({ frames, now }: { frames: SensorFrame[]; now: number }) {
  const end = Math.max(now, frames.at(-1)?.t ?? 0);
  return (
    <section className="panel">
      <h2>Cinco sensores · últimos 20 s (mm)</h2>
      <div className="legend sensor-legend">
        {SENSOR_DEFS.map((sensor) => (
          <span key={sensor.key}>
            <i style={{ background: `var(--${sensor.color})` }} />
            {sensor.short}
          </span>
        ))}
      </div>
      <svg viewBox="0 0 600 260" role="img" aria-label="Gráfica temporal de los cinco sensores">
        {[0, 100, 200, 300, 400].map((v) => (
          <g key={v}>
            <line x1="35" x2="590" y1={230 - v * 0.5} y2={230 - v * 0.5} stroke="var(--grid)" />
            <text x="0" y={235 - v * 0.5} fill="var(--muted)" fontSize="12">
              {v}
            </text>
          </g>
        ))}
        {SENSOR_DEFS.map((sensor) => (
          <path
            key={sensor.key}
            d={sensorPath(frames, sensor.key, end)}
            fill="none"
            stroke={`var(--${sensor.color})`}
            strokeWidth="2"
            strokeDasharray={sensor.key.length > 1 ? '6 3' : undefined}
          />
        ))}
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
