import { useState } from 'react';
import type { Cell } from '../telemetria/types';
// Modos del firmware (arquitectura v2): el numero es el que se envia con m<n> y s<n>.
export const modes = ['Resolver laberinto desconocido', 'Mapear, guardar y buscar ruta más corta', 'Resolver laberinto aprendido'];
export const modeHelp = [
  'Busca la meta desde cero sin reemplazar el mapa guardado.',
  'Explora todo lo alcanzable, vuelve a la salida, guarda el mapa y calcula la ruta más corta.',
  'Llega a la meta por caminos conocidos del mapa guardado. Requiere completar antes el modo 2.',
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
      setMode(2);
      localStorage.setItem('raton-mode', '2');
      return send('s2');
    },
    // Resolver hasta la celda elegida usando el mapa aprendido.
    route: (goal: Cell) => {
      setMode(3);
      localStorage.setItem('raton-mode', '3');
      return sequence([`GW${goal.join(',')}`, 's3']);
    },
    sequence,
  };
}
