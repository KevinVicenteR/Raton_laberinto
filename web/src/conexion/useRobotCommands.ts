import { useState } from 'react';
import type { Cell } from '../telemetria/types';
// Modos del firmware (arquitectura v2): el numero es el que se envia con m<n> y s<n>.
export const modes = ['Autónomo completo', 'Competencia', 'Camino más corto'];
export const modeHelp = [
  'Diagnostica mientras resuelve, mapea todo, regresa, guarda y resuelve el mapa aprendido.',
  'Prioriza llegar a la meta con diagnóstico y recuperación en marcha.',
  'Usa solo caminos confirmados del mapa guardado por el modo autónomo.',
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
      localStorage.setItem('raton-mode', '1');
      return send('s1');
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
