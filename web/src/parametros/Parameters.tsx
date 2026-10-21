import { useState } from 'react';
import { definitions, editedParameterCommands } from './parameterCommands';
export function Parameters({
  disabled,
  sequence,
  send,
  onApplied,
  boardParams,
}: {
  disabled: boolean;
  boardParams: Record<string, number>;
  sequence: (commands: string[]) => Promise<boolean>;
  send: (cmd: string) => Promise<boolean>;
  onApplied: (speed: number, range: number) => void;
}) {
  const [values, setValues] = useState<Record<string, string>>({}),
    [goal, setGoal] = useState(''),
    [corridor, setCorridor] = useState(''),
    [free, setFree] = useState(''),
    [error, setError] = useState('');
  const effective = (key: string, fallback: number) => values[key] ?? String(boardParams[key] ?? fallback);
  const apply = async () => {
    try {
      const commands = editedParameterCommands(values, goal);
      if (!commands.length) {
        setError('Modifica un ajuste antes de enviarlo.');
        return;
      }
      setError('');
      if (await sequence(commands)) {
        onApplied(+effective('searchSpeed', 400), +effective('sensorMaxMm', 600));
        setValues({});
        setGoal('');
      }
    } catch (error) {
      setError(String(error instanceof Error ? error.message : error));
    }
  };
  return (
    <section className="panel">
      <h2>Parámetros</h2>
      <fieldset disabled={disabled}>
        <p className="status">
          Solo se envían los campos que edites. Lee los valores de la placa para ver su configuración actual.
        </p>
        {definitions.map(([key, label, fallback, min, max, step]) => (
          <label className="field" key={key}>
            {label} <small>{key in boardParams ? '· placa' : '· referencia'}</small>
            {key === 'sideEnabled' ? (
              <select
                value={effective(key, fallback)}
                onChange={(e) => setValues({ ...values, [key]: e.target.value })}
              >
                <option value="0">Desactivado</option>
                <option value="1">Activado</option>
              </select>
            ) : (
              <input
                type="number"
                min={min}
                max={max}
                step={step}
                value={effective(key, fallback)}
                onChange={(e) => setValues({ ...values, [key]: e.target.value })}
              />
            )}
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
          <button disabled={Object.keys(values).length > 0 || !!goal} onClick={() => void send('save')}>
            Guardar en la placa
          </button>
          <button onClick={() => void send('params')}>Leer valores de la placa</button>
        </div>
        <p className="status">
          Laterales nuevos: XSHUT izquierdo A6 y derecho A3. Mide su posición y comprueba lecturas antes de
          activar el centrado.
        </p>
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
