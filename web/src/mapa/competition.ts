import mazeText from '../../../mazes/reglamento_2026.txt?raw';
export const competition = {
  W: 19,
  H: 10,
  PITCH: 155,
  WALL: 12, // mismo grosor nominal que Params::wallMm; el paso nominal es 155 mm
  start: [18, 0] as [number, number],
  goal: [4, 2] as [number, number], // contando desde abajo a la izquierda
  axleBack: 33, // el eje de ruedas va 33 mm por detras del centro del raton
  // Misma fuente que el simulador: evita divergencias de paredes entre web y firmware.
  map: mazeText.split(/\r?\n/).filter((line) => line.length > 0 && !line.startsWith('#')),
};
