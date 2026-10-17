import { useState } from 'react';
import type { Cell } from '../domain/types';
export const modes = ['Pared derecha', 'Pared izquierda', 'Laberinto', 'Ruta competencia', 'Espacio libre'];
export function useRobotCommands(send: (cmd: string) => Promise<boolean>) {
  const [mode, setMode] = useState(() => {
    const n = Number(localStorage.getItem('raton-mode'));
    return n >= 1 && n <= 5 ? n : 1;
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
    train: () => {
      setMode(3);
      return sequence(['G-1,-1', 's3']);
    },
    route: (goal: Cell) => {
      setMode(3);
      return sequence([`G${goal.join(',')}`, 's3']);
    },
    sequence,
  };
}
