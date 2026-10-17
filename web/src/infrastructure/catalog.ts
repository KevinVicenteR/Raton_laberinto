import type { Catalog } from '../domain/types';
import { validateReplay } from '../domain/protocol';

const noResults =
  'Todavía no hay resultados de simulación. Genera web/public/sim/dashboard.json con el simulador Python.';
export async function loadCatalog(): Promise<Catalog> {
  const response = await fetch('/sim/dashboard.json', { cache: 'no-store' });
  if (!response.ok) throw Error(noResults);
  let data: Catalog;
  try {
    data = (await response.json()) as Catalog;
  } catch {
    // Vite can return its HTML fallback when a public file does not exist.
    throw Error(noResults);
  }
  if (!data || !Array.isArray(data.results) || !Array.isArray(data.replays)) {
    throw Error('El catálogo de simulación no es válido.');
  }
  return data;
}
function simulationUrl(file: string) {
  const name = file.split('/').at(-1);
  if (!name || !name.endsWith('.json')) throw Error('Archivo de sesión no válido.');
  return `/sim/${encodeURIComponent(name)}`;
}
export async function fetchSession(file: string) {
  const response = await fetch(simulationUrl(file));
  if (!response.ok) throw Error('No se pudo cargar la sesión.');
  return validateReplay(await response.json());
}
