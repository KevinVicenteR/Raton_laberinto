import { useState } from 'react';
import type { Cell } from '../telemetria/types';
// Modos del firmware (arquitectura v2): el numero es el que se envia con m<n> y s<n>.
export const modes = [
  'Explorar (competencia)',
  'Carrera (competencia)',
  'Práctica (cualquier medida)',
  'Sensores',
  'Calibrar motores',
  'Prueba de ruedas',
  'Borrar mapa',
];
export const modeHelp = [
  'Pista de la competencia (celda cellMm, meta guardada): calibra los sensores en la salida, meta, ruta más corta, vuelta y guarda el mapa',
  'Pista de la competencia: ruta más rápida con el mapa guardado (curvas suaves)',
  'Cualquier pista: calibra y mide el pasillo en la salida, explora todo sin repetir y vuelve. No toca la competencia',
  'Lecturas de los láseres y el giroscopio',
  'Opcional (el ratón ya aprende sus motores solo): en el suelo, pasillo recto de 2 celdas, mide kV, kS y τ',
  'Ratón LEVANTADO: cada rueda adelante y atrás',
  'Olvida el mapa guardado en flash',
];
export function useRobotCommands(send: (cmd: string) => Promise<boolean>) {
  const [mode, setMode] = useState(() => {
    const n = Number(localStorage.getItem('raton-mode'));
    return n >= 1 && n <= modes.length ? n : 1;
  });
  const select = (n: number) => {
    setMode(n);
    localStorage.setItem('raton-mode', String(n));
    void send(`m${n}`);
  };
  const sequence = async (commands: string[]) => {
    for (const cmd of commands) if (!(await send(cmd))) return false;
    return true;
  };
  return {
    mode,
    select,
    start: () => send(`s${mode}`),
    stop: () => send('x'),
    restart: () => send('R'),
    eraseMap: () => send('erase'),
    loadMap: () => send('map'),
    train: () => {
      setMode(1);
      return send('s1');
    },
    // La celda elegida en el mapa aprendido (salida en 20,20) como meta, y carrera.
    route: (goal: Cell) => {
      setMode(2);
      return sequence([`GW${goal.join(',')}`, 's2']);
    },
    sequence,
  };
}
