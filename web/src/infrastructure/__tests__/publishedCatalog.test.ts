import { describe, expect, it } from 'vitest';
import type { Catalog } from '../../domain/types';
import { validateReplay } from '../../domain/protocol';

// Vite loads the shipped artifacts too: every listed replay must exist and validate.
const files = import.meta.glob('../../../public/sim/*.json', { eager: true, import: 'default' });
const catalog = files['../../../public/sim/dashboard.json'] as Catalog;

describe('catálogo publicado', () => {
  it('incluye una reproducción válida por cada resultado real', () => {
    expect(catalog.results.length).toBeGreaterThanOrEqual(20);
    expect(catalog.replays).toHaveLength(catalog.results.length);
    for (const [index, replay] of catalog.replays.entries()) {
      const data = files[`../../../public/${replay.file}`];
      expect(data, replay.file).toBeDefined();
      expect(() => validateReplay(data), replay.file).not.toThrow();
      expect(data).toMatchObject({
        mode: catalog.results[index].mode,
        reached: catalog.results[index].reached,
        clean_arrival: catalog.results[index].clean_arrival,
        contacts: catalog.results[index].contacts,
        corridor_mm: catalog.results[index].corridor_mm,
      });
    }
  });
});
