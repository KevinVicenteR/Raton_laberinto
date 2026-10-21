export const definitions = [
  ['searchSpeed', 'Velocidad explorando (mm/s)', 400, 50, 1500, 10],
  ['runSpeed', 'Velocidad en carrera (mm/s)', 650, 50, 2500, 10],
  ['turnSpeed', 'Velocidad en curvas (mm/s)', 450, 50, 1500, 10],
  ['cellMm', 'Celda: pasillo + pared (mm)', 155, 100, 400, 1],
  ['wallMm', 'Grosor de pared (mm)', 12, 0, 40, 1],
  ['kpRot', 'Rumbo Kp', 0.02, 0, 0.2, 0.001],
  ['kSteer', 'Centrado (grados por mm)', 0.25, 0, 2, 0.01],
  ['sensorMaxMm', 'Alcance láseres (mm)', 600, 100, 2000, 10],
  ['sideAheadMm', 'Laterales: distancia delante del eje (mm)', 0, -80, 80, 1],
  ['sideLateralMm', 'Laterales: distancia del centro a cada lente (mm)', 30, 0, 70, 1],
  ['sideLeftOffsetMm', 'Corrección lateral izquierdo (mm)', 0, -60, 60, 1],
  ['sideRightOffsetMm', 'Corrección lateral derecho (mm)', 0, -60, 60, 1],
  ['sideEnabled', 'Usar laterales para centrar y confirmar pasos', 0, 0, 1, 1],
] as const;

export function editedParameterCommands(edited: Record<string, string>, goal: string) {
  const commands: string[] = [];
  for (const [key, value] of Object.entries(edited)) {
    const definition = definitions.find((d) => d[0] === key);
    const n = Number(value);
    if (
      !definition ||
      !value.trim() ||
      !Number.isFinite(n) ||
      n < definition[3] ||
      n > definition[4] ||
      (['sideEnabled', 'sideLeftOffsetMm', 'sideRightOffsetMm'].includes(key) && !Number.isInteger(n))
    )
      throw new Error('Revisa los límites de los parámetros numéricos.');
    commands.push(`set ${key} ${value}`);
  }
  if (goal) {
    if (!/^\d+,\d+$/.test(goal) || goal.split(',').some((v) => +v > 31))
      throw new Error('Meta: x,y entre 0 y 31.');
    commands.push(`G${goal}`);
  }
  return commands;
}
