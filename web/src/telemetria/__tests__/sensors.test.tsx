import { describe, expect, it } from 'vitest';
import { renderToStaticMarkup } from 'react-dom/server';
import { initialState, telemetryReducer } from '../state';
import { currentReading, parseSensorLine, sensorPath } from '../sensorData';
import { Sensors, Chart } from '../Sensors';
import { parseRecording } from '../recording';
import { stateAt } from '../../simulacion/replayState';

const diagnostic = 'SENS I-1 F145 D108 fresh=0,0,1 giro=0 encI=0 encD=0';
const sides = 'LATERALES I-1 D91 fresh=0,0 control=0';
const nav = 'NAV I52 F100 D60 v300 r0 p143 x10 y20 a0 P1550 H900 G1';

describe('telemetría de cinco sensores', () => {
  it('muestra diagnósticos reales sin inventar pose ni actualizar el mapa', () => {
    let state = initialState();
    const occupancy = state.occupancy;
    state = telemetryReducer(state, { type: 'line', line: diagnostic, t: 100 });
    state = telemetryReducer(state, { type: 'line', line: sides, t: 120 });
    expect(state.sensorReadings).toEqual({
      I: { mm: -1, t: 100 },
      F: { mm: 145, t: 100 },
      D: { mm: 108, t: 100 },
      LI: { mm: -1, t: 120 },
      LD: { mm: 91, t: 120 },
    });
    expect(state.sideControl).toBe(false);
    expect(state.last).toBeNull();
    expect(state.trail).toHaveLength(0);
    expect(state.occupancy).toBe(occupancy);
    const html = renderToStaticMarkup(
      <Sensors last={state.last} readings={state.sensorReadings} now={120} sideControl={state.sideControl} />,
    );
    expect(html).toContain('Lateral izquierdo · 90°');
    expect(html).toContain('Diagonal izquierdo · 45°');
    expect(html).toContain('Lateral derecho · 90°');
    expect(html).toContain('91 mm');
    expect(html).toContain('Sin lectura');
    expect(html).toContain('desactivado');
    expect((html.match(/class="tile"/g) ?? []).length).toBe(5);
  });
  it('une NAV y LATERALES sin duplicar la trayectoria ni borrar la pose', () => {
    let state = telemetryReducer(initialState(), { type: 'line', line: nav, t: 100 });
    const pose = state.last;
    const occupancy = state.occupancy;
    state = telemetryReducer(state, { type: 'line', line: 'LATERALES I54 D90', t: 120 });
    expect(state.last).toBe(pose);
    expect(state.trail).toHaveLength(1);
    expect(state.occupancy).toBe(occupancy);
    state = telemetryReducer(state, { type: 'line', line: nav, t: 200 });
    expect(state.sensorReadings.LI?.mm).toBe(54);
    expect(state.sensorReadings.LI?.t).toBe(120);
    expect(state.sensorReadings.F?.t).toBe(200);
  });
  it('mantiene compatibles registros antiguos y conserva laterales al reproducir un TXT', () => {
    const old = stateAt(parseRecording(`0.100\t${nav}`), 0.1);
    expect(old.sensorReadings.LI).toBeUndefined();
    const replay = stateAt(parseRecording(`0.100\t${diagnostic}\n0.120\t${sides}`), 0.12);
    expect(replay.sensorReadings.LD?.mm).toBe(91);
    expect(replay.sideControl).toBe(false);
    expect(replay.last).toBeNull();
  });
  it('lee los cinco canales de la prueba y rechaza mensajes ajenos o incompletos', () => {
    const values = parseSensorLine('PRUEBA_MM DI=12 F=34 DD=56 LI=78 LD=-1', 100);
    expect(values?.readings.LI?.mm).toBe(78);
    expect(values?.readings.LD?.mm).toBe(-1);
    expect(parseSensorLine('LATERALES I54', 100)).toBeNull();
    expect(parseSensorLine('LATERALES izq=ok der=ok control=0', 100)).toBeNull();
    expect(parseSensorLine('LATERALES I9999999999999999999999 D1', 100)).toBeNull();
  });
  it('no mantiene un lateral como reciente por recibir solo NAV', () => {
    let state = telemetryReducer(initialState(), { type: 'line', line: sides, t: 100 });
    state = telemetryReducer(state, { type: 'line', line: nav, t: 2000 });
    expect(currentReading(state.sensorReadings, 'LD', 2000)).toBeUndefined();
    expect(currentReading(state.sensorReadings, 'F', 2000)?.mm).toBe(100);
    const html = renderToStaticMarkup(
      <Sensors last={state.last} readings={state.sensorReadings} now={2000} />,
    );
    expect(html).toContain('Sin datos recientes');
    expect(html).not.toContain('91 mm');
    state = telemetryReducer(state, { type: 'line', line: 'REINICIO', t: 2100 });
    expect(state.sensorReadings).toEqual({});
  });
  it('corta las curvas en lecturas inválidas o huecos de telemetría', () => {
    const frames = [
      { t: 100, readings: { LI: { mm: 54, t: 100 } } },
      { t: 200, readings: { LI: { mm: -1, t: 200 } } },
      { t: 300, readings: { LI: { mm: 58, t: 300 } } },
      { t: 4000, readings: { LI: { mm: 60, t: 4000 } } },
    ];
    const path = sensorPath(frames, 'LI', 4000);
    expect((path.match(/M/g) ?? []).length).toBe(3);
    expect(path).not.toContain('L');
    const html = renderToStaticMarkup(<Chart frames={frames} now={4000} />);
    expect(html).toContain('LD');
    expect(html).toContain('side-left');
    expect(html).toContain('side-right');
  });
});
