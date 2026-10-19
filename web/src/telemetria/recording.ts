import { validateReplay } from '../conexion/protocol';
import type { ReplayData } from './types';
import { isRemovedMessage } from '../telemetria/robot';
export function parseRecording(text: string): ReplayData {
  if (text.trim().startsWith('{')) return validateReplay(JSON.parse(text));
  const trace = text.split(/\r?\n/).flatMap((line) => {
    const m = /^(\d+(?:\.\d+)?)\t(.+)$/.exec(line);
    return m && !isRemovedMessage(m[2]) ? [{ t: +m[1], _line: m[2], filtered: { L: -1, F: -1, R: -1 } }] : [];
  });
  return validateReplay({ trace });
}
export async function loadRecording(file: File) {
  if (file.size > 20 * 1024 * 1024) throw Error('El archivo supera 20 MB.');
  return parseRecording(await file.text());
}
export function downloadRecording(lines: readonly string[]) {
  const url = URL.createObjectURL(
    new Blob(['segundos\tlinea\n' + lines.join('\n')], { type: 'text/plain;charset=utf-8' }),
  );
  const a = document.createElement('a');
  a.href = url;
  a.download = `raton-registro-${new Date().toISOString().replace(/[:.]/g, '-')}.txt`;
  a.click();
  setTimeout(() => URL.revokeObjectURL(url), 1000);
}
