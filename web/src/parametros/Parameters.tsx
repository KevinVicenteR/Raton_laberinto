import { useState } from 'react';
// Parametros del firmware v2 (mismo nombre que en lib/raton_core/src/raton/params.h). Se envian con
// "set nombre valor"; "save" los guarda en la flash de la placa.
const definitions = [
  ['searchSpeed', 'Velocidad explorando (mm/s)', 400, 50, 1500, 10],
  ['runSpeed', 'Velocidad en carrera (mm/s)', 650, 50, 2500, 10],
  ['turnSpeed', 'Velocidad en curvas (mm/s)', 280, 50, 1500, 10],
  ['cellMm', 'Celda: pasillo + pared (mm)', 155, 100, 400, 1],
  ['wallMm', 'Grosor de pared (mm)', 12, 0, 40, 1],
  ['kpRot', 'Rumbo Kp', 0.02, 0, 0.2, 0.001],
  ['kSteer', 'Centrado (grados por mm)', 0.25, 0, 2, 0.01],
  ['sensorMaxMm', 'Alcance láseres (mm)', 600, 100, 2000, 10],
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
    [goal, setGoal] = useState(''),
    [corridor, setCorridor] = useState(''),
    [free, setFree] = useState(''),
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
    if (goal && !/^\d+,\d+$/.test(goal)) {
      setError('Meta: x,y (por ejemplo 4,2).');
      return;
    }
    setError('');
    const commands = definitions.map(([key]) => `set ${key} ${values[key]}`);
    if (goal) commands.push(`G${goal}`);
    if (await sequence(commands)) onApplied(+values.searchSpeed, +values.sensorMaxMm);
  };
  return (
    <section className="panel">
      <h2>Parámetros</h2>
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
          Meta x,y (desde abajo a la izquierda)
          <input
            value={goal}
            placeholder="4,2"
            onChange={(e) => setGoal(e.target.value.replace(/\s/g, ''))}
          />
        </label>
        <div className="row">
          <button onClick={() => void apply()}>Enviar ajustes</button>
          <button onClick={() => void send('save')}>Guardar en la placa</button>
          <button onClick={() => void send('params')}>Ver valores</button>
        </div>
        <hr />
        <p className="status">
          La calibración de paredes es opcional y no bloquea el arranque. El ratón aprende motores y
          giroscopio en marcha (se guarda solo). A mano, si quieres: ratón en el centro de una celda con pared
          a los dos lados.
        </p>
        <button
          onClick={() => {
            if (
              window.confirm(
                '¿Ratón en el suelo, en el centro de una celda con pared a los dos lados? Va a girar.',
              )
            )
              void send('calsonda');
          }}
        >
          Calibrar sensores (gira en el sitio)
        </button>
        <button onClick={() => void send('calfront')}>Solo frontal (centrado, mirando a una pared)</button>
        <label className="field">
          Ancho del pasillo (mm)
          <input
            type="number"
            min="100"
            max="400"
            value={corridor}
            onChange={(e) => setCorridor(e.target.value)}
          />
        </label>
        <button
          onClick={() => {
            if (!corridor || +corridor < 100 || +corridor > 400) {
              setError('Pasillo: entre 100 y 400 mm.');
              return;
            }
            void send(`calside ${corridor}`);
          }}
        >
          Fijar pasillo y calibrar (gira en el sitio)
        </button>
        <button
          onClick={() => {
            if (window.confirm('¿Ratón levantado, con ruedas al aire? Girará cada rueda 0,5 s.'))
              void send('T');
          }}
        >
          Prueba de ruedas
        </button>
        <hr />
        <label className="field">
          Orden libre (set, get, estado, map…)
          <input value={free} placeholder="set kpRot 0.02" onChange={(e) => setFree(e.target.value)} />
        </label>
        <button
          onClick={() => {
            if (free.trim()) void send(free.trim());
          }}
        >
          Enviar orden
        </button>
      </fieldset>
      {error && <p role="alert">{error}</p>}
    </section>
  );
}
