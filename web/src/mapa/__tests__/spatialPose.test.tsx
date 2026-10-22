import { describe, expect, it } from 'vitest';
import { renderToStaticMarkup } from 'react-dom/server';
import { parseRobot } from '../../telemetria/robot';
import { liveMaze } from '../liveMaze';
import { spatialPose, spatialSegments } from '../spatialPose';
import { LiveMap2D } from '../LiveMap';
import { emptyOccupancy, updateOccupancy } from '../occupancy';

const maze = liveMaze({ mazeW: 19, mazeH: 10, startX: 18, startY: 0 }, {});
const nav = (x: number, y: number, t: number, suffix = 'P1550 H900 G1') =>
  parseRobot(`NAV I50 F100 D50 v180 r0 p143 x${x} y${y} a0 ${suffix}`.trim(), t)!;

describe('marco espacial continuo', () => {
  it('conserva el origen absoluto si NAV pierde P/H/G', () => {
    expect(spatialPose(nav(2867, 77, 0), maze)).toEqual([2867, 77]);
    expect(spatialPose(nav(2867, 77, 100, ''), maze)).toEqual([2867, 77]);
  });
  it('normaliza el paso aprendido de cada muestra', () => {
    expect(spatialPose(nav(2960, 80, 0, 'P1600 H900 G1'), maze)).toEqual([2867.5, 77.5]);
  });
  it('no recorta una pose exterior para hacerla parecer válida', () => {
    const outside = nav(4000, 77, 100);
    expect(spatialPose(outside, maze)).toBeNull();
    expect(outside.x).toBe(4000);
    const html = renderToStaticMarkup(<LiveMap2D maze={maze} cells={{}} trail={[outside]} last={outside} />);
    expect(html).not.toContain('translate(4000');
  });
  it('corta la trayectoria en saltos, poses inválidas y desconexiones', () => {
    const trail = [
      nav(2867, 77, 0),
      nav(2867, 95, 100),
      nav(4000, 77, 200),
      nav(2867, 110, 300),
      nav(1000, 110, 400),
      nav(1000, 130, 500),
      nav(1000, 150, 3000),
    ];
    expect(spatialSegments(trail, maze)).toEqual([
      [
        [2867, 77],
        [2867, 95],
      ],
      [
        [1000, 110],
        [1000, 130],
      ],
    ]);
  });
  it('no fabrica paredes a partir de ecos tomados mientras gira', () => {
    const grid = emptyOccupancy();
    expect(updateOccupancy(grid, nav(2867, 77, 0, 'P1550 H900 G0'))).toBe(grid);
    expect(updateOccupancy(grid, nav(4000, 77, 0), 600, {}, { mazeW: 19, mazeH: 10 })).toBe(grid);
  });
});
