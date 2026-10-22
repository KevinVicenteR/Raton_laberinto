> Actualización 2026-10-21: el menú tiene únicamente 1 Resolver desconocido, 2 Mapear/guardar/ruta mínima, 3 Resolver aprendido. Los números arrancan directamente; `s1`–`s3` y `m1`–`m3` siguen disponibles. La calibración de paredes es opcional (`calsonda`) y no forma parte del arranque. Las referencias posteriores a los siete modos o práctica corresponden al diseño anterior. Véase README para el funcionamiento actual.

# Traspaso: estado del trabajo y cómo seguir

Documento para quien continúe (persona, Codex u otro agente). Resume qué se pidió, qué está hecho, cómo se
mide y qué falta. **Hablar siempre en español con el usuario.**

## Contexto

- Ratón de laberinto para SECIHTI 2026 (categoría "Robot ratón"): Arduino Nano 33 BLE Sense Rev2, 3 × VL53L0X
  (frontal y dos a 45°), DRV8833 con motores N20, giroscopio BMI270.
- Rama de trabajo: **`arquitectura_v2`**. No tocar ni fusionar en `sin_arrancador` (lo pidió el usuario).
- Lo que pide el usuario: que el ratón sea **autónomo y autosuficiente** (que se calibre y ajuste solo, en la
  competencia no se puede tocar) y **rápido** resolviendo pistas. Quejas vistas en el ratón real: señales
  falsas (en un pasillo vacío los sensores "ven algo delante"), retroceso y giro de calibración poco fluidos,
  se pegaba a las paredes, zigzag y no reconocía por dónde ya pasó.
- Medidas aproximadas del ratón (por fotos): láseres a ~40–50 mm del piso, metidos detrás de agujeros en una
  carcasa negra impresa en 3D; ruedas cerca de las esquinas delanteras con cola larga; menos de 10 cm. Faltan
  las medidas reales (ver `docs/calibracion.md`, "Medir una vez con una regla").

## Cómo compilar y medir

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j4
build/raton_tests                      # 324 comprobaciones, deben dar 0 fallos (~10 s)
scripts/banco.sh 80                    # 81 casos de competencia con errores de nivel 2 (~2,5 min, 4 núcleos)
SIM_H=25 scripts/banco.sh 80 --set sensorHeightMm=25   # sensores bajos
SIM_MOTOR=0.8 scripts/banco.sh 80      # motores un 20 % más lentos (batería baja)
build/raton_sim --practica 70          # pistas de práctica de cualquier medida (~10 min, 1 núcleo)
pio run                                # firmware (PlatformIO, ver platformio.ini)
```

Un caso concreto del banco con todo el registro: el caso `k` del banco es
`build/raton_sim --random <1000+k> --perturb 2 --seed <77+k> --simseed <5+k>` (añadir `--trace` para la pose
cada 0,1 s). Variables de diagnóstico (solo en el simulador, `RATON_DEBUG`):
- `SIM_CONTACTLOG=1`: errores de pose en cada choque;
- `SIM_LOCLOG=1`: error de pose real;
- `RADARDBG=<t>`: correcciones de pose del radar desde t durante 1,5 s;
- `VOTEDBG=<seg>`: votos de una pared (seg = `y*64+x` horizontal, `4096+y*64+x` vertical; ver `maze.h`);
- `CELLDBG=1`: aprendizaje del paso de celda;
- `PIVDBG=1`: estado al acabar cada giro en el sitio.

Errores del simulador: `SIM_H` (altura real de los láseres), `SIM_MOTOR`, `SIM_FLOOR` (reflejo del piso),
`SIM_NOVERT` (sin errores en lo vertical), `SIM_SOLO=<grupo>`; opciones `--fuera <mm>` (objetos fuera del
laberinto), `--sincal`, `--cono <grados>`.

## Resultados de referencia

Banco grande (`scripts/banco.sh 160`, 161 casos, láseres a 40 mm), tras la tanda de rapidez: 161 cumplen,
149 sin roce, meta explorando 63,8 s, exploración completa 118 s, carrera 16,4 s. Con 81 casos: láseres a
25 mm 81/81 (77 sin roce); motores al 80 % 81/81 (74). **Con 81 casos el resultado varía bastante con
cualquier cambio (el sistema es caótico): para decidir, 161 casos.**

### Banco de 81 casos antes de la tanda de rapidez

| Configuración | Cumplen | Sin roce | Paredes mal | Error de pose (avance/lateral/rumbo) |
|---|---|---|---|---|
| Láseres a 40 mm | 81 | 77 | 3 | 4,6 / 2,7 mm / 0,84° |
| Láseres a 25 mm | 80 | 77 | 8 | 4,6 / 2,6 mm / 0,86° |
| Motores al 80 % | 81 | 78 | 5 | 4,3 / 2,6 mm / 0,85° |

Antes de los cambios de esta tanda (con el modelo 3D del sensor ya en el simulador): 78/72, 67/61 y 72/54.
Tiempos medios con láseres a 40 mm: meta explorando 69 s, exploración completa 127 s, carrera 17 s.
Práctica (`--practica 70`): 70 de 70 pistas exploradas completas.

### Tanda de fluidez y precisión (después de la de rapidez)

161 casos: 161 cumplen, 155 sin roce, 2 paredes mal; meta 63,4 s, completa 117 s, carrera 16,0 s. Láseres a
25 mm y motores al 80 %: 80/81 cada uno (una carrera fallida en cada uno: casos 1046 y 1069).
- Rozamiento (`kS`) en rampa en vez de a saltos; retroceder y enderezarse a la vez tras un atasco.
- Calibración por barrido continuo (sin paradas): ~2 mm de error, unos 4 s.
- Paso de celda por regresión en cada recta + Kalman entre rectas: converge al valor real (±0,5 mm). Antes
  quedaba 3–7 mm corto y en las rectas largas de la carrera la curva empezaba 4 cm antes.
- Escala del giroscopio aprendida más despacio (8 % por giro, antes 30 %): antes bailaba hasta un 4 %.

Descartado: curva de la carrera guiada por lo recorrido en vez de por el tiempo (arregla el caso 1069 pero en
conjunto 160/161 y más zigzag); estimar el ángulo de montaje de los laterales y lo torcido en la salida con
el barrido (salían peor que no estimarlos).

Nota: `scripts/banco.sh` tenía una carrera al juntar resultados (a veces contaba casos de más, p. ej.
"casos 153" con 81); ya está arreglado.

### Rapidez: probado

Dónde se va el tiempo explorando hasta la meta (caso 1005, 78 s): 21 s girando en el sitio, 5 s parado, 15 s
acelerando y frenando. En la carrera (24 s), 14 s en curvas a ~290 mm/s: la manda la rueda de fuera de
la curva, que llega a lo que da el motor.

Se quedó:
- radio de curva elegido por tiempo con los motores reales y `turnSpeed` solo como tope: carrera −4 %;
- `knownSpeed` (650): crucero rápido por rectas de celdas ya recorridas: exploración completa −4 %;
- empezar el giro en el sitio con el ratón casi parado (< 40 mm/s): −1,5 %.

Descartado (en 161 casos):
- `searchSpeed 500`: meta −11 %, pero fallos y roces en carrera;
- `runSpeed 900`: no cambia nada (ya manda el tope real de los motores, ~600 mm/s);
- coste de giro de la ruta 12 o 16 (en vez de 8): igual o peor;
- 95 % del motor para la rueda de fuera en curva (en vez de 85 %): −3 % de carrera pero muchos más roces;
- giros en el sitio más rápidos (630–720°/s): −4 % pero más zigzag y paredes mal;
- curvas sin parar explorando por celdas ya conocidas (`--smooth` para todo, o solo en lo conocido):
  −6 % a −13 % de tiempo, pero el mapa empeora mucho (150–370 paredes mal) y fallan carreras;
- girar con el ratón la posición y sus dudas en cada giro (el avance pasa a lateral): en teoría correcto,
  en la práctica 158/161. La duda de rumbo proporcional al ángulo girado: 157/161;
- corregir los láseres con el paso de celda aprendido (`rangeBiasFromCell 1`): se realimenta con el
  aprendizaje del paso y supone el grosor de pared nominal.

## Hecho en esta tanda (sin subir aún si este documento no está en el último commit)

1. **Modelo 3D del VL53L0X** en el simulador y en la sonda: piso, paredes de altura finita, lo que pasa por
   encima. Integración exacta en el ángulo vertical (sin sesgo de 15 a 45 mm de altura).
2. **Duda por el piso** en cada predicción, que se usa en las tolerancias del radar y en la pose.
3. **Aprendizaje en marcha** de la inclinación de cada láser y del reflejo del piso; se guarda al terminar.
4. **¿Pared o piso?** (`frontSeesWall`) en paradas por pared inesperada, atascos, comprobación tras girar y
   colocación con el frontal. Detector de "empuja la pared" solo por debajo de donde empieza el piso.
5. **Paso de celda aprendido en todos los modos** (pista que no mide lo nominal, ruedas) y corrección de los
   desfases de los láseres con ese paso (`rangeBias`).
6. **Motores saturados:** la consigna no se escapa (ya no hay atascos falsos a toda velocidad), velocidad
   máxima real con el modelo aprendido, y curvas de carrera limitadas a lo que da la rueda de fuera.
7. **Error en los giros en el sitio:** lo que patinaban las ruedas al girar se sumaba al lateral con el rumbo
   nuevo (2–3 cm de error tras una media vuelta). Corregido.
8. `PISTA_DISTINTA` lo confirman los laterales (un frontal sin calibrar ya no la dispara).
9. Firmware: guarda la inclinación y el piso aprendidos; compila.

## Pendiente (por orden)

1. **Rapidez** (petición del usuario): lo que queda es menos paradas al explorar. Las curvas sin parar en
   lo conocido ganan mucho tiempo pero empeoran el mapa: habría que entender por qué (pose tras la curva,
   votos con la pose dudosa) antes de activarlas. También: rutas en diagonal en la carrera.
2. **Movimiento fluido:** hecho lo principal (rozamiento en rampa, retroceso en un movimiento, calibración
   por barrido). Queda: el recentrado en "S" tras un atasco son aún 4 movimientos con parada.
3. **Autoprueba al arrancar** (motores, encoders, giroscopio, láseres) y seguir funcionando con un láser roto.
4. **Chasis rectangular con eje adelantado** en el simulador (`Physical::rectBody` ya existe) cuando el usuario
   dé las medidas; comprobar que las curvas y los giros caben.
5. Casos que aún fallan: con láseres a 25 mm el caso 1046 choca en la carrera; mirar con `SIM_CONTACTLOG`.
6. Actualizar la tabla de resultados del README con un banco completo (`build/raton_sim --bench 160`, ~10 min).


## Continuación: fallos de sensores (20 de octubre de 2026)

Implementado sobre `bb60027`:

- Inyección reproducible en el simulador: `SIM_FAIL_LEFT`, `SIM_FAIL_FRONT`, `SIM_FAIL_RIGHT` y
  `SIM_FAIL_GYRO` indican el segundo desde el arranque en que dejan de llegar muestras. Valor negativo
  desactiva. `SIM_FREEZE_GYRO=1` junto con `SIM_FAIL_GYRO` simula muestras nuevas con ángulo congelado.
  Los fallos no cambian la física ni los sensores restantes.
- Un láser sin muestras durante 150 ms se excluye: se borra la lectura retenida y la memoria de proximidad,
  sin tratar la desconexión como espacio libre. Un lateral puede volver a incorporarse al recibir muestras.
  Los timeouts de transporte del VL53L0X ya no se publican como lecturas fuera de alcance.
- Con un lateral ausente continúa usando los otros sensores. Sin el frontal o sin ambos laterales, pone
  ambos motores a cero y termina la misión. Intentar continuar sin frontal produjo demasiados contactos.
- La IMU sin muestras durante 100 ms, un ángulo no finito, o una salida prácticamente congelada durante
  más de 200 ms pese a más de 20° medidos por encoders enclavan la parada. Una muestra recuperada no
  vuelve a activar los motores; hace falta iniciar explícitamente una nueva misión (`begin/reset`).
- La parada se propaga a los giros, barridos, muestreo y navegación. Una misión terminada por fallo no
  guarda el mapa parcial ni intenta regresar a casa; la calibración interrumpida no aplica sus resultados.

Ejemplos (el resultado `FALLO` es esperado si el robot se detiene por pérdida de IMU o frontal):

```bash
SIM_FAIL_LEFT=10 build/raton_sim --random 1000 --perturb 2 --seed 77 --simseed 5
SIM_FAIL_FRONT=10 build/raton_sim --random 1000 --perturb 2 --seed 77 --simseed 5
SIM_FAIL_GYRO=10 SIM_FREEZE_GYRO=1 build/raton_sim --random 1000 --perturb 2 --seed 77 --simseed 5
```

Verificación: práctica 70/70; CTest, 324 comprobaciones sin fallos; compilación CMake de `firmware_check` con stubs.
Banco de competencia nivel 2: 161/161, 155 sin roce, 2 paredes mal; meta 63,4 s, exploración 117,3 s,
carrera 16,0 s. Caso 1000 con fallo lateral izquierdo/derecho a los 10 s: termina exploración y carrera,
sin contactos ni paredes mal. Pérdida del frontal: parada a los 10,150 s, sin contactos; IMU congelada:
parada a los 10,346 s, sin contactos. Esto no demuestra tolerancia a cualquier avería ni equivale a
pruebas físicas en el robot.

Pendientes que NO se consideran resueltos: autoprueba física de motores y encoders; láser que publica
una distancia falsa constante con muestras nuevas; continuidad segura sin frontal; curvas de exploración,
rutas diagonales, recentrado en S y los casos extremos 1046/1069. No se activaron las optimizaciones
que el traspaso anterior documenta como descartadas. Para el chasis real siguen faltando medidas.

Compilación para la placa con PlatformIO: no completada en este entorno; la instalación quedó en
`platformio/toolchain-gccarmnoneeabi`. La comprobación con stubs no sustituye `pio run` ni una prueba física.
Publicación pendiente: Git no tiene credenciales de escritura y el conector devolvió 403.

## Autocalibración por rueda — 21 de octubre de 2026

Parte de `854a912` y conserva las correcciones de arranque/sensores y Bluetooth aportadas por Kevin. No cambia los tres modos. Antes de moverse valida el sesgo del giroscopio con muestras suficientes, finitas y estables y comprueba reposo de encoders; reintenta tres veces y cancela si no hay estabilidad.

En marcha añade trim diferencial lento del modelo de motor y apoyo de arranque independiente para cada rueda/sentido. El apoyo se acota a 0,22 y solo se conserva tras confirmar respuesta del encoder. Se aplica también al error final significativo de un giro. Encoders no finitos o movimiento sostenido contrario a la salida aplicada provocan parada enclavada. No adivina ni cambia polaridades de cableado.

Guarda el paso efectivo aprendido solamente con al menos tres regresiones informativas y varianza menor de 4 mm². Reutiliza el paso si coinciden celda configurada, diámetro de rueda y cuentas por vuelta. No identifica diámetro físico ni relación de engranajes sin referencia externa. Inclinación/piso y escala del giroscopio conservan el aprendizaje con paredes seguras. Los offsets absolutos de distancia siguen necesitando una referencia (`calsonda` opcional).

Los nuevos parámetros se añaden al final de Params y el cargador acepta también el tamaño anterior con CRC válido, conservando la configuración existente. No hay escrituras flash durante el control; se guarda al terminar sin fallo de sensores.

Validación: 364 comprobaciones del núcleo (incluyen sesgo inestable/NaN, ayuda de arranque sin guardar bloqueos, encoder invertido, invalidación de paso guardado, asimetría de motores y giro lento con rozamiento 0,24, completado dentro de los 2 s del pivote). Firmware comprobado en PC con stubs. Banco nominal 161/161, 157 sin contacto, 3 paredes mal, carrera 15,8 s; base del día 161/161, 158 sin contacto, 5 paredes mal, carrera 15,8 s. Contactos de carrera: 68 base frente a 38 nuevos. Sensores bajos 81/81 (79 sin contacto), motores al 80 % 81/81 (81 sin contacto).

El trim inicial al 10 % por segundo fallaba un caso (1046): se redujo al 2 % por segundo y se repitió todo el banco. La compilación real con `pio run` para `nano33ble` finalizó correctamente: RAM estática 179584/262144 bytes (68,5 %), flash 464444/983040 bytes (47,2 %). Falta cargarla y probarla físicamente; el simulador no valida el cableado ni el par real.


## Recuperación de atascos y perfil físico — 21 de octubre de 2026

El informe de la pista real indica parada tras atascarse, desvíos en recta/giro y contactos frecuentes. No se dispone todavía del registro NAV de ese intento físico; estos cambios corrigen fallos reproducidos en el control y se deben comprobar en el robot.

Cambios:

- La discrepancia entre encoders y ángulo con muestras de IMU válidas se trata como bloqueo/patinaje recuperable. No se diagnostica IMU averiada solo con ese desacuerdo. Pérdida de muestras, NaN y encoders en sentido opuesto mantienen parada enclavada.
- Un pivote no cambia rumbo/celda si no alcanzó el ángulo. `GIRO_INCOMPLETO` solicita rescate y un nuevo intento. Los giros también pueden recuperarse al salir de la celda inicial.
- El rescate desactiva temporalmente el centrado lateral y las actualizaciones del mapa con pose incierta: antes el centrado podía contradecir la alineación, manteniéndolo a unos 5° del rumbo y consumiendo los intentos. Comprueba tiempos, alineación y posición antes de anunciar `RECUPERADO`.
- Retroceso limitado al interior de la celda y al espacio conocido detrás del cuerpo, con 12 mm de margen. Tres intentos consecutivos por celda; avanzar a otra reinicia el contador. Si el bloqueo es permanente, `RECUPERACION_AGOTADA` termina la misión sin guardar mapa parcial ni aprendizaje de motor de ese intento.
- Ante una lectura de poste con descentrado, no se cierra directamente un paso conocido: se recentra y se vuelve a comprobar. El frontal mantiene su función de parada.
- Velocidad/derivadas y temporización de atasco usan el intervalo real. `hold` no convierte una muestra repetida en una velocidad cero ficticia.
- `precisionMode=1` por defecto, también migrando los dos formatos anteriores de parámetros. Máximo 300 mm/s de avance, 360 °/s y 1800 °/s² de giro, 1500 mm/s² de aceleración y 2000 mm/s² de frenado (se respetan valores inferiores). Sin curvas en carrera; prima control sobre tiempo. No añade modos al menú.

Pruebas nuevas: ciclos de 2/6/30 ms; rescate tras bloqueo temporal en pasillo estrecho empezando descentrado 18 mm sin contacto; giro parcial bloqueado y llegada a la celda física correcta; bloqueo permanente acotado a tres intentos sin guardar; separación entre ausencia de IMU y desacuerdo de giro. Total 378 comprobaciones.

La compilación real con PlatformIO para Nano 33 BLE pasa. Los límites del perfil son parte del comportamiento que se prueba: los tiempos de carrera de la sección previa corresponden a la versión rápida anterior. Las simulaciones calibran los offsets con paredes de referencia; en la placa, `calsonda` es opcional y debe ejecutarse centrado entre paredes laterales si los offsets no se han medido. Conservar las dimensiones reales del cuerpo/ruedas/sensores. Falta el registro de telemetría y comprobación física de esta corrección.

Resultado final de esta corrección: **378/378 comprobaciones**, firmware Nano 33 BLE compilado (179600 bytes RAM, 467092 bytes flash). Banco nominal **161/161**, 157 sin contacto, 16 paredes mal acumuladas; meta media 88,4 s, exploración 166,0 s, carrera 42,3 s; 6 contactos en carrera. Motores al 80 %: **81/81**, 76 sin contacto. Sensores a 25 mm: **80/81**, fallo de carrera en aleatorio 1046 (sí alcanza meta explorando); pendiente. No presentar ese escenario como resuelto ni trasladar los resultados del simulador a una garantía de precisión física. El perfil rápido (`precisionMode=0`) no es la configuración verificada para esta entrega.


## Web de cinco sensores y protección de calibración — 21 de octubre de 2026

Se muestran los cinco canales en tarjetas, radar y gráfico: DI/F/DD y LI/LD a 90°. Se interpretan NAV,
SENS, LATERALES y PRUEBA_MM sin añadir poses ficticias. Los datos inválidos y los antiguos se distinguen.
NAV publica LATERALES también con sideEnabled=0, indicando por separado si participan en el control.
No se activa automáticamente la fusión de los laterales con geometría todavía sin medir. Cableado:
XSHUT original A0/A1/A2; lateral izquierdo A6, derecho A3; bus SDA A4/SCL A5.

La web ya no manda todos sus valores de referencia al editar un campo: únicamente los ajustes editados.
Puede leer la configuración real de la placa. El mapa de competencia usa la misma transcripción que el
simulador; las paredes 3D comparten marco con NAV v2 y se escalan con el paso aprendido. Se elimina el
desfase de salida que afectaba al mapa 3D en vivo y el grosor fijo de 15 mm (nominal actual: 12 mm).
Esto no verifica la transcripción pared por pared contra la pista física del evento.

Calibración: calside no modifica la celda antes de validar el arranque; un barrido fallido o un error al
guardar conserva parámetros y mapa anteriores. Un barrido exitoso tampoco borra el mapa en RAM.
calfront rechaza muestras inestables, geometría incoherente y offsets fuera de límites; anuncia guardado
solo si la escritura fue exitosa. La persistencia de aprendizaje restaura los parámetros anteriores si falla
flash. No se cambia el formato de Params ni Maze. Se descartó también cambiar el conteo de muestras
frontales al terminar un pivote: alteraba la temporización y hacía fallar el caso 1046 con tres sensores.

Se compararon estrategias de replanificación al confirmar aperturas y un suavizado adicional del mando
de centrado. Se descartaron: no mejoraron el conjunto y aumentaron los contactos. La búsqueda y el
control conservan la versión probada; no presentar esta entrega como eliminación del zigzag físico.
Base comprobada en 161 casos: tres sensores 161/161, 157 sin contacto, 491 contactos acumulados
(6 en carrera); cinco sensores activos 161/161, 160 sin contacto, 8 contactos acumulados (8 en carrera).
Los contactos acumulados son pasos de simulación en contacto, no impactos físicos independientes.
Tiempos medios base con cinco sensores: meta 88,1 s, exploración 165,8 s, carrera 42,4 s.
La variante de replanificación daba 156 sin contacto y 306 contactos con cinco sensores: no se publica.

Validación de esta entrega: 408 comprobaciones del núcleo, compilación de firmware con stubs de PC,
43 pruebas web, TypeScript/build de producción y ESLint. Se corrigió la estructura del renderizador de
paredes del solucionador adaptable para que ESLint compruebe los hooks sin el bucle de dibujo.
La compilación Nano con PlatformIO vuelve a quedar bloqueada descargando framework-arduino-mbed;
no atribuir a esta entrega una compilación real completada. No hay puerto serie conectado: faltan carga,
prueba física de cinco canales y medición de montaje. Sigue pendiente el caso extremo con sensores
bajos 1046 registrado antes.


## Cruces y dos salidas laterales simultáneas

Los dos canales de 90° mantienen evidencia independiente, sin else-if entre izquierda y derecha.
Confirmar una apertura no borra la otra. En una T, cruz o bifurcación se conserva cada borde del mapa;
el planificador compara las cuatro direcciones, incluida la vuelta cuando corresponde. Un dato
desconocido puede ser candidato durante exploración, pero no habilita una ruta aprendida como abierta.
Una lectura inválida o repetida no confirma una apertura. En mapeo completo, las ramas alcanzables
siguen siendo objetivos aunque primero se elija el otro lado. En modo meta no se exige visitar todas
las ramas antes de llegar: ese cometido corresponde al modo de mapeo.

Se añaden regresiones exhaustivas de los 81 estados locales (abierto/cerrado/desconocido en cuatro
direcciones) con cuatro rumbos: 324 configuraciones, cada una con objetivos en los cuatro vecinos,
exploración optimista y rutas conocidas. Se comprueba que ambas salidas simultáneas sobreviven a
la pérdida de una lectura lateral. Se añaden dos misiones físicas simuladas, T y cruz, con sensores
laterales activos: mapear ambos extremos laterales, volver, guardar y resolver el mapa aprendido.
El cambio no modifica las ganancias, calibración, temporización ni decisiones del control probado.

Validación: 6939 comprobaciones del núcleo, 0 fallos, incluyendo las dos misiones de cruces.

## Rama experimental de autonomía v3

Trabajo autorizado por Kevin sobre `experimental/autonomia-v3`, base `ea7bd38`.
El menú experimental ahora es 1 Autónomo completo, 2 Competencia y 3 Camino
más corto. No fusionar en `arquitectura_v2` como parte de esta entrega.
La especificación y los límites están en [autonomia-v3.md](autonomia-v3.md).

Se añaden diagnóstico y contraste espacial durante navegación, detección de
oscilación, recuperación láser acotada, objetivos de exploración estables,
trayectorias condicionadas por geometría y un flujo completo sin reinicializar
la pose entre fases. La web simplificada muestra mapas 2D/3D y cinco canales;
reconecta al robot autorizado y solicita `sync` durante la marcha. El diario
retiene 32 eventos en RAM y comunica pérdidas; no es un registro flash completo.

No afirmar posición exacta, reparación de averías físicas ni éxito garantizado.
Las visitas del mapa son estimadas; POSE publica contraste externo e incertidumbre.
No habilitar geometría/fusión lateral sin medir el montaje real.

### Corrección de autoprueba tras calibrar y arranque degradado

El usuario reportó que los tres modos se cancelaban con
`AUTOPRUEBA_SENSORES_O_REPOSO` y solo funcionaba `T`. Se corrigió el uso de
una muestra `Board::last()` anterior a los 600 ms de calibración como comienzo
de la ventana de 250 ms. La función compartida `checkStartup()` toma una nueva
base y tiene regresiones de ese intervalo, muestras ausentes, datos no finitos,
movimiento con retorno a origen y cancelación.

La autoprueba es informativa para incidencias recuperables: se arranca con
límites de 120 mm/s y 180 °/s y se conserva el aprendizaje guardado anterior.
La calibración inestable puede reutilizar el sesgo anterior. `x`, reloj inválido
y encoders no finitos mantienen cancelación. No se eliminan los vigilantes de
averías críticas durante navegación. Modo 3 sin mapa válido inicia mapeo y
resolución; no se inventa una ruta aprendida.

