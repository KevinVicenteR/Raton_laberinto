import { useState } from 'react';
const definitions = [
  ['v', 'Velocidad (mm/s)', 100, 30, 400, 10],
  ['p', 'Dirección: 0 cascada / 1 PID', 0, 0, 1, 1],
  ['k', 'Centrado cascada', 0.15, 0, 0.3, 0.01],
  ['j', 'PID Kp', 0.0014, 0, 0.02, 0.0001],
  ['d', 'PID Kd', 1, 0, 5, 0.1],
  ['r', 'Alcance láseres (mm)', 300, 60, 400, 10],
  ['g', 'Eco mínimo', 0.35, 0.1, 3, 0.05],
  ['w', 'Umbral lateral (mm)', 100, 40, 400, 1],
  ['z', 'Punto muerto (mm)', 40, 0, 150, 1],
] as const;
export function Parameters({
  disabled,
  sequence,
  send,
  onApplied,
}: {
  disabled: boolean;
  sequence: (commands: string[]) => Promise<boolean>;
  send: (cmd: string) => Promise<boolean>;
  onApplied: (speed: number, range: number) => void;
}) {
  const [values, setValues] = useState<Record<string, string>>(
      Object.fromEntries(definitions.map(([key, , value]) => [key, String(value)])),
    ),
    [weights, setWeights] = useState('1.0,0.8,0.3,0.2'),
    [goal, setGoal] = useState(''),
    [cell, setCell] = useState(''),
    [calibration, setCalibration] = useState(''),
    [error, setError] = useState('');
  const apply = async () => {
    if (
      definitions.some(
        ([key, , , min, max]) =>
          values[key].trim() === '' ||
          !Number.isFinite(+values[key]) ||
          +values[key] < min ||
          +values[key] > max,
      )
    ) {
      setError('Revisa los límites de los parámetros numéricos.');
      return;
    }
    const w = weights.split(',').map(Number);
    if (
      w.length !== 4 ||
      w.some((n) => !Number.isFinite(n)) ||
      (goal && !/^-?\d+,-?\d+$/.test(goal)) ||
      (cell && (!Number.isFinite(+cell) || +cell < 100 || +cell > 400))
    ) {
      setError('Revisa pesos, meta y tamaño de celda (100–400 mm).');
      return;
    }
    setError('');
    const commands = definitions.map(([key]) => `${key}${values[key]}`);
    commands.push(`n${w.join(',')}`);
    if (goal) commands.push(`G${goal}`);
    if (cell) commands.push(`c${Math.round(+cell)}`);
    if (await sequence(commands)) onApplied(+values.v, +values.r);
  };
  return (
    <section className="panel">
      <h2>Parámetros en marcha</h2>
      <fieldset disabled={disabled}>
        {definitions.map(([key, label, , min, max, step]) => (
          <label className="field" key={key}>
            {label}
            <input
              type="number"
              min={min}
              max={max}
              step={step}
              value={values[key]}
              onChange={(e) => setValues({ ...values, [key]: e.target.value })}
            />
          </label>
        ))}
        <label className="field">
          Pesos: espacio, novedad, giro, recto
          <input value={weights} onChange={(e) => setWeights(e.target.value)} />
        </label>
        <label className="field">
          Meta x,y
          <input
            value={goal}
            placeholder="4,2"
            onChange={(e) => setGoal(e.target.value.replace(/\s/g, ''))}
          />
        </label>
        <label className="field">
          Celda (mm; vacío: automática)
          <input type="number" min="100" max="400" value={cell} onChange={(e) => setCell(e.target.value)} />
        </label>
        <div className="row">
          <button onClick={() => void apply()}>Enviar ajustes</button>
          <button onClick={() => void send('?')}>Ver valores</button>
        </div>
        <hr />
        <label className="field">
          Pasillo medido (100–400 mm; vacío: medir)
          <input
            type="number"
            min="100"
            max="400"
            value={calibration}
            onChange={(e) => setCalibration(e.target.value)}
          />
        </label>
        <p className="status">Para calibrar, coloca el ratón centrado con paredes a ambos lados.</p>
        <button
          onClick={() => {
            if (calibration && (+calibration < 100 || +calibration > 400)) {
              setError('Pasillo: entre 100 y 400 mm.');
              return;
            }
            void send(`K${calibration}`);
          }}
        >
          Calibrar quieto
        </button>
        <button
          onClick={() => {
            if (window.confirm('¿Ratón levantado, con ruedas al aire? Girará cada rueda durante 0,6 s.'))
              void send('T');
          }}
        >
          Prueba de ruedas
        </button>
      </fieldset>
      {error && <p role="alert">{error}</p>}
    </section>
  );
}
