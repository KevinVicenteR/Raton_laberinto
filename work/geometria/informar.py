"""Genera el informe exclusivamente a partir de resultados completos."""
import json,re
from collections import Counter
from pathlib import Path
D=Path(__file__).resolve().parent
cal=json.loads((D/'calibracion_final.json').read_text())
a=json.loads((D/'suite21.json').read_text());b=json.loads((D/'reserved.json').read_text())
assert a['complete'] and b['complete'] and len(a['results'])==21 and len(b['results'])==180
best=cal['selected']
def metrics(data):
    rr=data['results']; arrived=[r for r in rr if r['reached'] is not None]
    return dict(n=len(rr),clean=sum(r['clean_arrival'] for r in rr),arrivals=len(arrived),
        contacts=sum(r['contacts'] for r in rr),mean=sum(r['elapsed'] for r in rr)/len(rr),
        mean_arrivals=sum(r['reached'] for r in arrived)/len(arrived) if arrived else None,
        reasons=dict(Counter(r['failure'] for r in rr if r['failure'])))
x,y=metrics(a),metrics(b)
pistas=(D/'sim_pistas.log').read_text().splitlines()
pclean=parr=pcontacts=0;times=[]
for line in pistas:
    contact=re.search(r'choques\s+(\d+)',line)
    if not contact:continue
    contacts=int(contact[1]);pcontacts+=contacts
    arrival=re.search(r'llega en\s+([\d.]+) s',line)
    if arrival:
        parr+=1;pclean+=contacts==0;times.append(float(arrival[1]))
assert len(pistas)==21
assert 'OK' in (D/'tests.log').read_text()
lines=['# Geometría real y navegación por celdas', '',
f"Modelo y controladores actualizados; 27 pruebas aprobadas. Resultado final mapped: {x['clean']}/21 llegadas limpias en suite 21 y {y['clean']}/180 en reservada. Frente a la referencia, la variación es {x['clean']-11:+d} y {y['clean']-80:+d} llegadas limpias, respectivamente. Las limitaciones del control siguen reflejadas en las tablas de fallos.", '',
'## Cambios', '',
'Robot por defecto de 80 × 80 mm, eje central, nariz a 40 mm; sensores laterales en (30, ±30) mm y rayos a ±45°. `Robot.preset("antiguo")` conserva el cuerpo y montajes anteriores. `firmware_config()` importa las constantes de geometría, resuelve expresiones aritméticas y usa los valores nuevos como respaldo cuando faltan.', '',
'Los giros de celdas son en el sitio, con realimentación angular hasta ±0,5° del rumbo de cuadrícula y `pending=0`. Los callejones usan 180° y avance hacia delante. La celda inicial se escanea con el frontal tras girar izquierda 90°, derecha 180° y volver 90°. Las demás reciben las observaciones laterales capturadas durante su aproximación; el frontal se clasifica parado con umbral bruto de 100 mm.', '',
'El centrado convierte las lecturas con `raw*sin(45°) + 30 - 40`, con objetivo lateral de 30 mm. La parada frontal de última celda usa el mayor entre 30 mm y zona muerta más margen. Ante avance de cero celdas se conserva la pared frontal, se vuelve a observar una vez y se cuentan intentos consecutivos; el tercero produce `sin avance`. Si la inundación ya no encuentra ruta, puede terminar antes. Al reobservar tras un fallo se conservan las direcciones absolutas de las paredes laterales; no se reutilizan izquierda/derecha del rumbo anterior como si fueran del nuevo. Hay una regresión específica para esa asociación.', '',
'`ObservedMapSim` comprueba el barrido del cuerpo nuevo, con envolvente de 3 mm y pasos angulares de 1°, usando únicamente paredes observadas/prior explícito y odometría. No ajusta la entrada ni retrocede para girar. El banco `sim_pistas` usa exploración por celdas también para modo 5 con meta; conserva el simulador base, mientras `mapped` añade el guardia de mapa observado. El guardia nativo antiguo, que supone sensores perpendiculares y giro de dos tiempos, no se aplica a la geometría diagonal del simulador base.', '',
'## Calibración sin usar mapas de evaluación', '',
'Cada configuración se probó en las semillas **1000–1019**, modo 3, ruido 1, más el mapa de competencia en modo 4. Modo 5 con meta usa el mismo explorador; las evaluaciones posteriores sí incluyen ambos modos. Primero se probaron las nueve combinaciones de umbral/anticipación {50, 70, 90} mm con parada nominal de 30 mm; se comprobaron márgenes de parada de 25 y 35 mm en 70/70, y después las nueve combinaciones con parada de 35 mm. La opción de 25 queda limitada físicamente a 30 mm por la fórmula nominal. Son 19 configuraciones y 399 episodios completos.', '',
'La selección maximiza llegadas limpias, después llegadas, después minimiza contactos y tiempo medio. Es el mejor punto del barrido discreto, no una garantía de óptimo continuo. Cuando varios puntos empatan exactamente se conserva el primero probado; 70/70 mantiene los valores nominales actuales del firmware.', '',
f"**Valores elegidos: CELL_SIDE_WALL_MM={best['threshold']:g} mm; SIDE_LOOK_AHEAD_MM={best['look']:g} mm; parada frontal={best['effective_stop']:g} mm.** En entrenamiento: {best['clean']}/{best['n']} llegadas limpias, {best['arrivals']} llegadas y {best['contacts']} contactos.", '',
'La parada de 35 mm exige un margen de 25 mm sobre la zona muerta de 10 mm y queda 5 mm antes del centro ideal cuando hay pared frontal. El modelo de sensor mantiene mínimo fiable de 30 mm, cono de 25°, ruido de 3 mm y pérdidas del 2 %: parar a 30 mm resulta frágil. Este resultado depende de ese modelo y no es una medida del sensor real.', '',
'| Umbral lateral | Anticipación | Parada solicitada | Limpias/21 | Llegadas | Contactos |',
'|---:|---:|---:|---:|---:|---:|']
for r in cal['summary']:
    lines.append(f"| {r['threshold']} | {r['look']} | {r['stop']} | {r['clean']} | {r['arrivals']} | {r['contacts']} |")
lines += ['', '## Evaluaciones finales', '',
'Parámetros congelados antes de ejecutar las suites 21 y reservada. La referencia anterior procede de la tarea; no se volvió a ejecutar la versión antigua.', '',
'| Banco | Versión | Limpias | Llegadas | Tiempo medio por episodio (s) | Contactos |',
'|---|---|---:|---:|---:|---:|',
'| Suite 21, mapped | Anterior, referencia | 11/21 | 17/21 | No facilitado | 12 |',
f"| Suite 21, mapped | Geometría real calibrada | {x['clean']}/21 | {x['arrivals']}/21 | {x['mean']:.2f} | {x['contacts']} |",
'| Reservada, mapped | Anterior, referencia | 80/180 | 114/180 | 70 (criterio previo no especificado) | 88 |',
f"| Reservada, mapped | Geometría real calibrada | {y['clean']}/180 | {y['arrivals']}/180 | {y['mean']:.2f} | {y['contacts']} |", '',
f"Entre episodios que llegan, la media de llegada es {x['mean_arrivals']:.2f} s en suite 21 y {y['mean_arrivals']:.2f} s en reservada. La media global incluye fallos tempranos y no debe interpretarse sola como velocidad de resolución. Contactos cuenta episodios de contacto registrados por la física, no fotogramas bloqueados. Una llegada es entrada física en la celda objetivo, no llegada exacta a su centro.", '',
f"`sim_pistas.py`: **{pclean}/21 limpias, {parr}/21 llegadas, {pcontacts} contactos**, tiempo medio de las llegadas {sum(times)/len(times):.2f} s (el CLI no informa duración de los fallos). Véase `sim_pistas.log` para cada pista.", '',
'## Fallos pendientes', '',
'| Motivo | Suite 21 | Reservada |','|---|---:|---:|']
for reason in sorted(set(x['reasons'])|set(y['reasons'])):
    lines.append(f"| {reason} | {x['reasons'].get(reason,0)} | {y['reasons'].get(reason,0)} |")
lines += ['',
'Persisten errores de clasificación de paredes y diferencias entre posición física y odometría. Un barrido rechazado describe la decisión del guardia observado, no demuestra por sí solo una colisión real. `navegacion terminada sin llegada fisica` indica que el explorador terminó sin que la física registrase la meta, por ejemplo al agotar rutas en su mapa. No se añadió localización con coordenadas reales ni se ajustaron parámetros con las suites de evaluación.', '',
'## Pruebas y reproducción', '',
'**27 pruebas Python aprobadas**, incluida `test_sim_nets.py`, sin modificar ese archivo. La biblioteca existente en `build/` fallaba en dos pruebas de inferencia por estar desactualizada; se recompiló desde los fuentes actuales dentro del directorio permitido y se usó `RATON_NATIVE_LIB=$PWD/work/geometria/libraton_navigation.dylib`.', '',
'```sh',
'clang++ -std=c++17 -O2 -shared -fPIC -Ilib/raton_core/src sim/navigation_bridge.cpp lib/raton_core/src/motion_safety.cpp lib/raton_core/src/risk_predictor.cpp lib/raton_core/src/nets.cpp -o work/geometria/libraton_navigation.dylib',
'export RATON_NATIVE_LIB=$PWD/work/geometria/libraton_navigation.dylib',
'python3 -B work/geometria/calibrar.py',
'python3 -B work/geometria/refinar.py',
'sh work/geometria/evaluar.sh',
'```', '',
'`evaluar.sh` exporta `RATON_CELL_SIDE_WALL_MM`, `RATON_SIDE_LOOK_AHEAD_MM` y `RATON_CELL_FRONT_STOP_MM` con los valores seleccionados, ejecuta `sim_pistas.py`, las dos evaluaciones `mapped` con `--baseline tools/sim_extremos.py` y las pruebas `python3 -B -m unittest discover -s tests -p "test_sim_*.py"`. Sin esas variables, el simulador sigue las constantes actuales de `src/config.h`; no se modificaron para imponer la calibración.', '',
'Fuentes de resultados: `calibracion_final.json`, `suite21.json`, `reserved.json`, `sim_pistas.log` y `tests.log`. Los JSON de evaluación contienen huellas de fuentes y resultados por episodio; `configuracion.json` conserva los valores base y sus huellas. No se tocaron firmware ni telemetría, no hubo commit/push ni carga al robot.', '']
(D/'informe.md').write_text('\n'.join(lines))
(D/'resumen.json').write_text(json.dumps(dict(selected=best,suite21=x,reserved=y,sim_pistas=dict(clean=pclean,arrivals=parr,contacts=pcontacts)),indent=2))
print(json.dumps(dict(selected=best,suite21=x,reserved=y),indent=2))
