# Control y predictor de riesgo

El objetivo es llegar sin contactos. Una parada, un giro incompleto o agotar el tiempo son fallos,
aunque reduzcan el contador de contactos. El hardware sigue siendo Nano 33 BLE, tres VL53L0X,
encoders y giroscopio. No se modificaron sus dimensiones ni montaje.

## Estado de activación

`motion_safety.h`: `kMotionGuardEnabled=false`. El verificador de barrido es experimental y su
paquete de control no superó la referencia en las pruebas. En firmware se observa y registra,
sin modificar la maniobra. `risk_model.h`: modelo entrenado 43–32–16–1, con
`kRiskModelEnabled=false`. Su salida se registra; no selecciona PWM ni puede cancelar movimientos
en el estado entregado. Los cuatro pesos anteriores siguen siendo una puntuación de navegación,
no los pesos del nuevo modelo neuronal.

Los resultados experimentales distinguen `original` (simulador conservado antes del trabajo),
`corrected` (filtro de mediana, giro de 3 s y barrido observado con correcciones de entrada) y
`neural` (ese mismo control más un veto experimental). No equivalen a tres firmwares cargados.
El original conserva su límite interno de giro de 4 s y recuperación simulada; la configuración
numérica importable, incluido KP_WALL 0.15, es común. No se alteró la referencia para ocultar fallos.

## Construcción y pruebas

Requiere CMake, compilador C++17 y Python 3.9+. Solo entrenar requiere numpy; inferencia y pruebas
no necesitan PyTorch, TensorFlow ni numpy. PlatformIO compila el firmware, sin cargarlo.

```sh
cmake -S . -B build
cmake --build build -j 4
ctest --test-dir build --output-on-failure
pio run -e raton
```

Python utiliza el núcleo C++ mediante `build/libraton_navigation.dylib` (macOS), `.so` (Linux) o
`raton_navigation.dll` (Windows). Para otra carpeta de compilación, definir `RATON_NATIVE_LIB` con
la ruta completa. Esta dependencia es intencional: el barrido y la inferencia nativa que prueban
los bancos son los mismos que se compilan para el robot. CTest configura esta ruta automáticamente.

## Experimentos reproducibles

Guardar primero la versión de referencia del simulador antes de modificarlo. `--baseline` recibe
ese archivo; la entrega incluye la copia usada, con su hash en el informe. No usar una copia nueva
del simulador modificado como referencia del trabajo anterior.

```sh
python3 -B tools/sim_pistas.py
python3 -B tools/navigation_experiments.py collect --out work/navigation-data.json --budget 6000
python3 -B tools/navigation_experiments.py train --data work/navigation-data.json \
  --out tools/models/risk_model.json --export lib/raton_core/src/risk_model.h --budget 1200
cmake --build build -j 4
python3 -B tools/navigation_experiments.py calibrate --model tools/models/risk_model.json \
  --out work/validation-navigation.json
python3 -B tools/navigation_experiments.py evaluate --baseline work/baseline-simulator.py \
  --model tools/models/risk_model.json --out work/navigation-evaluation.json --suite all
python3 -B tools/navigation_replays.py --model tools/models/risk_model.json --out work/replays
```

La generación tiene un límite de tiempo real y termina procesos pendientes al agotarlo. El
entrenamiento descuenta de dos horas el tiempo registrado de generación. No se presenta una
generación incompleta como completa. Los resultados de evaluación se guardan progresivamente
con `complete=false` hasta terminar. No se activa automáticamente un modelo al entrenar.

Entrenamiento: mapas 1000–1039. Validación de modelo y umbrales: mapas 1100–1109. Prueba reservada:
mapas 200–229, modos 3 y 5, ruido 1–3. Los 21 casos anteriores se conservan. La selección de umbral
por navegación exige más llegadas sin contacto, sin perder llegadas ni éxitos limpios previos.
La primera validación de navegación no encontró ningún umbral mejor. La segunda red tampoco
propone un umbral con la sensibilidad de clasificación solicitada (`threshold=null`). Ninguno
de esos resultados autoriza activar el modelo.

## Contratos y límites

- `MotionResult`: completado, distancia real, error angular y motivo. Se conserva la interfaz del
  solver por celdas; los avances confirmados y abortos se registran sin inventar desplazamientos.
- `assessTurn`: recibe tres observaciones locales y la envolvente existente de 83/17/92 mm.
  No recibe el mapa verdadero. Comprueba cuerpo completo, margen experimental de 3 mm, segmentos
  locales de 30 mm y ajustes de entrada de hasta 15 mm. `observed=false` significa que no hay
  evidencia suficiente: `feasible=true` en ese caso **no certifica** ausencia de obstáculos.
- `riskFeatures`: tres tiempos × tres sensores × (distancia, validez, confianza, edad), velocidad,
  velocidad angular, error de rumbo, residuo y dirección en tres bits: 43 valores. La normalización
  y el orden son comunes entre Python y C++; no entran coordenadas verdaderas ni paredes ocultas.
- La verdad física del simulador solo produce lecturas, etiquetas y métricas/reproducciones.
  El modelo ve ejemplos de maniobras ejecutadas; no hay exploración contrafactual exhaustiva.
- `MOTION`, `OBS`, `SWEEP` y `RISK` amplían la telemetría sin sustituir las líneas de la interfaz
  anterior. `OBS` se limita a 4 Hz y las líneas de fin de maniobra incluyen lecturas crudas y filtradas,
  edad y confianza. La cola sigue sin bloquear el control; `drops` revela líneas perdidas por presión.
  Sin conexión no se graba una sesión persistente en el robot.

La mediana del simulador no reproduce el Kalman, PWM, fricción ni sensores reales con exactitud.
El mínimo fiable de 30 mm es una hipótesis del simulador, no una especificación universal del VL53L0X.
La salida sigmoid es un indicador aprendido, no una garantía probabilística de seguridad física.
Antes de activar algo hay que superar validación y prueba reservada sin alterar los umbrales con
los resultados reservados. La etapa física queda pendiente y no forma parte de estos experimentos.

## Referencias de diseño

- https://micromouseonline.com/2012/05/15/improved-search-turns/ — repetibilidad de giros, con video.
- https://ukmars.org/contests/micromouse/ — videos de búsqueda y carrera en competencia.
- https://micromouseonline.com/2011/10/12/using-the-front-sensors-for-triggering-turns/ — referencia frontal.
- https://github.com/ukmars/mazerunner-core — separación de sensores, perfiles y motores.
- https://www.st.com/resource/en/datasheet/vl53l0x.pdf — documentación del sensor.
- https://community.st.com/mems-sensors-48/minimum-range-vl053l0-54599 — límites de corta distancia.
- https://github.com/fomalhaut251/micromouse-gazebo-drl — referencia experimental, no un puerto directo.

No se copió código de los repositorios investigados. No se reactivaron los ajustes rechazados en
la búsqueda anterior (`robust_lateral`, `fused_stop`, `adaptive_split`, `align_front`, etc.). El nuevo
barrido observado se evaluó separadamente y quedó desactivado tras las regresiones encontradas.


## Segunda ronda: páginas y control experimental

Abrir `tools/simular.sh simulacion` para ver el laboratorio y `tools/simular.sh web` para
telemetría. Ambas páginas usan `tools/sim_resultados/dashboard.json`. Los recorridos guardados
se reproducen con sensores, antigüedad, contactos y resultado final; no son simulaciones
inventadas en JavaScript. Telemetría permite importar JSON o registros descargados de la propia
página. En reproducción se bloquean órdenes al robot y el supervisor automático. Seleccionar
un modo ya no reinicia el robot. Los diagnósticos OBS/MOTION/RISK/SWEEP se interpretan también
en conexión real. Si falla la vista 3D, el resto de la página sigue disponible.

Los siguientes controladores son **candidatos del simulador**, no implementaciones activadas
ni cargadas en el firmware:

- `precision-guard`: conserva 70°–avance–20°, con una corrección giroscópica lenta hasta 0.5°
  antes del avance. Conserva el error de actuación grueso, añade perturbación en la corrección
  y tiene un límite adicional de 1 s. Se conservan ruido, pérdidas y geometría física.
- `hybrid`: usa lo anterior y, en modo 5 con meta explícita, utiliza memoria de celdas. Esto
  cambia la estrategia del modo 5; no equivale a mejorar sus cuatro pesos.
- `hybrid-neural`: ordena alternativas con el predictor. `ranked` conserva la marcha atrás
  como último recurso, porque la red no fue entrenada para estimar su riesgo.
- `mapped`: comprueba el cuerpo completo contra paredes ya observadas y, únicamente en modo
  4, contra el mapa previo explícito de competencia. Integra encoders y giroscopio para ese
  control: nunca lee `self.x`, `self.y`, `self.segs` ni las paredes ocultas para decidir el barrido.
  Las correcciones de entrada de hasta 30 mm son movimientos físicos simulados, no cambios
  de coordenadas. Una pared observada se representa conservadoramente como un tramo de celda.
- `survey` y `mapped-raw`: experimentos con 20 observaciones estacionarias, filtradas o crudas.
  Más observaciones no garantiza mejores recorridos; sus resultados también se conservan.

Resultados de 21 casos: barrido inicial 0 llegadas limpias / 76 contactos; giro preciso con
barrido 4 / 28; memoria y giro preciso 6 / 24; barrido con memoria de paredes 11 / 12.
Los 21 casos conservan los límites anteriores (120, 300 o 600 s según el caso). El objetivo
sigue pendiente: **11/21 no es cero fallos**. Los mapas reservados se reportan por separado.

```sh
python3 -B tools/navigation_experiments.py evaluate --controllers mapped --suite all \
  --baseline work/baseline-simulator.py --out work/mapped.json
python3 -B tools/navigation_experiments.py collect --training-controller mapped-raw \
  --noises 1,2,3,4 --budget 1800 --out work/mapped-data.json
python3 -B tools/navigation_experiments.py train --data work/mapped-data.json \
  --out tools/models/checkpoints/candidate.json --positive-weight 4 --budget 1200
python3 -B tools/publish_dashboard.py --evaluation work/mapped.json \
  --model tools/models/risk_model.json --controller mapped
node tests/test_telemetry_protocol.cjs
```

El publicador solo acepta evaluaciones completas y vuelve a ejecutar las reproducciones;
rechaza publicarlas si llegada, contactos o fallo no coinciden con el resultado medido.
El ajuste `positive-weight` es experimental y no activa el modelo. Los modelos de rondas
anteriores se conservan en `tools/models/checkpoints/`. La comparación de modelos requiere
validación por navegación, además del error de clasificación.


Resultados completos del candidato `mapped`: 80/180 llegadas limpias en mapas 200–229
(114 llegadas, 88 contactos); 60/180 en mapas nuevos 400–429 (108 llegadas, 140 contactos).
Los mapas nuevos no se usaron para entrenar ni elegir el candidato. Las pruebas aisladas
obtuvieron 8/12 llegadas limpias y conservaron los fallos con descentrado, reversa y zigzag.
No se ha conseguido el objetivo ni existe una garantía de ausencia de contactos.

La tercera red usa 11.696 ejemplos de entrenamiento y 2.100 de validación después de quitar
duplicados exactos, procedentes de 600 episodios generados en esta ronda. En navegación de
validación, `mapped-ranked` empata 10/40 llegadas limpias con `mapped`, pero aumenta los contactos
20→24; no cumple el criterio de mejora. Queda en `tools/models/checkpoints/round3-candidate.*`,
desactivada. No sustituye al modelo de observación del firmware. Su C++ exportado coincide con
Python (100 entradas, error absoluto máximo 2.16e-7). Para repetir esa comprobación:

```sh
python3 -B tools/verify_risk_model.py --model tools/models/checkpoints/round3-candidate.json \
  --header tools/models/checkpoints/round3-candidate.h --workdir work/model-check
python3 -B tools/navigation_experiments.py validate-ranking \
  --model tools/models/checkpoints/round3-candidate.json --out work/validation-round3.json
python3 -B tools/navigation_experiments.py evaluate --controllers mapped --suite fresh \
  --baseline work/baseline-simulator.py --out work/new-maps.json
```

Las variantes `survey`, `mapped-raw`, la ordenación neuronal y la corrección con memoria de
paredes se mantienen accesibles para reproducir los experimentos; su presencia no significa
que superen a la mejor variante. No se transfirieron al robot los controladores del simulador.
El banco usa giroscopio y encoders idealizados, una geometría 2D y un modelo hipotético de
lecturas cortas; faltan calibración física, deslizamiento, dinámica de motores y validación real.

En las variantes con `goal_grid`, los modos 3 y 5 con meta comparten el explorador de celdas y
pueden repetir exactamente la trayectoria con la misma semilla. Los bancos cuentan ambas
entradas de modo, no 180 trayectorias estadísticamente independientes. La deduplicación del
conjunto de entrenamiento evita contar de nuevo los ejemplos exactos compartidos.
