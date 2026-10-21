# Calibración y aprendizaje automático

El perfil `precisionMode 1` está activado por defecto, también al cargar parámetros de las versiones anteriores. Limita el avance a 300 mm/s y la aceleración a 1500 mm/s²; los giros se hacen en el sitio, a un máximo de 360 °/s y 1800 °/s². El frenado usa hasta 2000 mm/s² (o menos si configuraste un límite inferior). Estos topes también se aplican a la carrera mientras se comprueba el robot físico. No borra tu geometría ni los ajustes aprendidos.

Ante un atasco intenta retroceder, alinear y replanificar hasta tres veces seguidas en la misma celda. Avanzar a otra celda reinicia ese contador. El retroceso se limita si conoce una pared detrás. Un giro incompleto no cambia el rumbo del mapa: informa `GIRO_INCOMPLETO` e intenta recuperarse. `RECUPERADO` indica que retoma la marcha; `RECUPERACION_AGOTADA` indica que no consiguió moverse tras los intentos. No se guarda un mapa parcial por ese fallo.

Durante el retroceso y la maniobra de rescate se suspende el centrado lateral y la escritura de paredes con una pose todavía incierta; así no contradicen el giro de alineación ni corrompen el mapa.

Una discrepancia entre ruedas y ángulo con muestras válidas puede ser patinaje o bloqueo: se trata como atasco, no se declara avería de IMU solo por ello. La pérdida de muestras, NaN o respuesta opuesta de encoder siguen causando parada enclavada. El barrido de calibración se cancela si no puede verificar el giro. El controlador y el temporizador de atasco usan el tiempo real entre lecturas.

Los tres modos son: **1 Resolver desconocido**, **2 Mapear y guardar**, **3 Resolver aprendido**. Todos usan el mismo control y aprendizaje. Coloca el ratón en el centro de la salida, con el rumbo configurado, y envía `1`, `2` o `3`. `x` detiene también durante la preparación.

## Antes de moverse

El robot mantiene los motores parados y calibra el sesgo del giroscopio durante 600 ms. Acepta la medida solo si recibe al menos 25 muestras finitas, su dispersión es menor de 0,35 °/s, el sesgo está dentro de ±10 °/s y ninguno de los encoders cambia más de dos cuentas. Reintenta hasta tres veces. Si no logra reposo, informa `CAL_NO_ESTABLE` y cancela el arranque; deja el robot quieto y vuelve a iniciar. Las comunicaciones siguen atendidas durante la calibración.

No exige paredes visibles para comenzar. Una muestra nueva fuera de alcance no equivale a un láser desconectado.

## Mientras recorre el laberinto

| Ajuste | Referencia utilizada | Qué conserva |
|---|---|---|
| Potencia común de los motores | Corrección necesaria en rectas estables | Modelo de velocidad `kV` |
| Diferencia entre motores | Error de rumbo, giroscopio y encoders durante rectas estables | `motorBalance`, limitado a ±25 % |
| Ayuda al arranque por rueda y sentido | Orden de movimiento sin respuesta inicial del encoder y movimiento posterior confirmado | `startBoostLF/LR/RF/RR`, hasta 0,22 de PWM adicional |
| Distancia por celda en unidades del encoder | Regresión de correcciones con paredes/postes a lo largo de rectas | `learnedCellMm`, solo tras tres rectas informativas y varianza menor de 4 mm² |
| Escala del giroscopio | Correcciones de rumbo con paredes después de giros | `gyroScale`, con cambios graduales |
| Inclinación de láseres y reflejo del piso | Lecturas comparadas con paredes ya confirmadas | Inclinaciones individuales y `floorReflect` |

La ayuda de arranque aumenta gradualmente cuando hay una orden de avance/giro o un error final significativo y la rueda no responde. Desaparece al ganar velocidad. No genera una maniobra independiente. Un bloqueo sin movimiento posterior no se guarda como rozamiento aprendido. El controlador conserva los límites de PWM y la detección de atascos.

Si el encoder indica movimiento sostenido en sentido opuesto a la orden aplicada, o un valor no finito, aparece `FALLO_ENCODER` y la parada queda enclavada para ese intento. Esto señala que debe revisarse cableado, polaridad o configuración; no invierte automáticamente los motores para adivinar la causa.

Al completar una misión sin fallo de sensores se guardan los ajustes en flash (meta alcanzada en 1/3; mapa completo y vuelta en 2). Si se interrumpe o no consigue salir de un atasco, se recuperan los ajustes de motor previamente guardados. `APRENDIDO` indica el resultado del guardado y `AUTOCAL` muestra el balance, las cuatro ayudas y el paso aprendido. No se escribe flash en cada ciclo. El paso guardado se reutiliza únicamente con la misma celda configurada, diámetro de rueda y cuentas por vuelta. Cambiar esos parámetros invalida su uso. El sesgo del giroscopio se vuelve a medir en reposo en cada arranque.

## Datos que necesita como referencia

Los pulsos del encoder por sí solos no permiten conocer milímetros reales ni separar diámetro de rueda, patinaje y tamaño de celda. El aprendizaje corrige el paso efectivo; no deduce ni modifica automáticamente el diámetro físico o la relación de la caja reductora.

Mide/configura una vez:

- `wheelDiamMm`, `countsPerRev` y `trackMm`; `firmware/config.h` contiene polaridad y asignación de motores/encoders.
- `cellMm` = pasillo + grosor de pared, `wallMm`, `mazeW` y `mazeH`.
- `startX`, `startY`, `startDir`; dirección 0=norte, 1=este, 2=sur, 3=oeste.
- `goalX`, `goalY`, `goalW` y `goalH`; `G 4,2` cambia la meta. El robot no tiene detector de la línea del suelo.
- Altura y posición de láseres (`sensorHeightMm`, `frontAheadMm`, `diagAheadMm`, `diagLateralMm`) y dimensiones del cuerpo.

Para comprobar los encoders sin movimiento automático, envía `sensores`, levanta el robot y gira cada rueda a mano: debe cambiar su encoder correspondiente. `x` sale del diagnóstico. Un giro manual del robot hacia la izquierda debe aumentar el ángulo del giroscopio. Corrige los signos en `firmware/config.h` si no coinciden.

## Desfases de distancia de los láseres

Los desfases absolutos necesitan una distancia de referencia. No se confunden con la inclinación y el reflejo del piso que sí se aprenden en marcha. El barrido `calsonda` es opcional: coloca el robot centrado entre paredes laterales de la geometría configurada; gira a cada lado, ajusta las lecturas y guarda si obtiene una medida válida. No se ejecuta obligatoriamente al enviar 1–3.

También existen `calfront` (robot centrado mirando una pared) y `calside <ancho del pasillo>`. Los offsets son `frontOffsetMm`, `leftOffsetMm` y `rightOffsetMm`. Una carcasa que tapa el cono o un sensor mal cableado requiere corregir el montaje.

## Consultar y restablecer

Todas las órdenes se envían por Bluetooth o Serie a 115200. `params` lista valores; `get <nombre>` consulta; `set <nombre> <valor>` cambia; `save` guarda cambios manuales. `learnMotor 1`, `learnGyro 1` y `learnVertical 1` están activados por defecto (usa `set` antes del nombre). `defaults` restablece todos los parámetros, incluida la geometría: configura otra vez la pista antes de correr.

Esta actualización conserva la configuración flash del formato anterior y añade los ajustes nuevos con valores neutros. `erase` borra el mapa, no es necesario para calibrar. Los tres modos del robot permanecen iguales.

## Dos laterales opcionales a 90 grados

Se conservan el frontal y los dos diagonales con su montaje y pines actuales. Los nuevos VL53L0X se leen
como `sideLeft`/`sideRight` y no sustituyen los diagonales. El usuario asigno `XSHUT_SIDE_LEFT=A0` y `XSHUT_SIDE_RIGHT=A4` en `firmware/config.h`.
Esta asignacion entra en conflicto con el diagonal izquierdo (A0) y SDA (A4). La comprobacion
`sidePinsConflict()` impide accionar los XSHUT nuevos y muestra `CONFIG_LATERALES_CONFLICTO`.
Aunque compile, **esta asignacion no habilita el funcionamiento de cinco sensores**. Si los modulos
ya estan conectados compartiendo esos cables, tampoco se garantiza que los originales inicialicen:
el software no puede separar dos dispositivos cuyo XSHUT esta fisicamente unido. Se necesita resolver
el conflicto de cableado antes de usar los cinco. La comprobacion tambien admite `-1` para omitir
sensores nuevos o pines libres cuando se acuerde la asignacion definitiva.
**No conectar los nuevos al bus sin tener controlados sus XSHUT**: todos arrancan en 0x29 y pueden
interferir con los sensores originales. Una vez asignados los pines, se inicializan individualmente con
las direcciones 0x33 y 0x34. Revisar que los pines elegidos no tengan otra funcion.

El control nuevo esta desactivado por defecto (`sideEnabled=0`). Con los nuevos instalados, primero usar
`sensores` para comprobar `LATERALES I... D...`, sin movimiento. Medir y configurar `sideAheadMm`
(distancia longitudinal de las lentes al eje de las ruedas, positiva delante) y `sideLateralMm` (del
centro a la lente, a cada lado). El modelo exige montaje simetrico y horizontal, apuntando a +-90 grados.
Los valores iniciales de geometria son provisionales, no mediciones del robot. Corregir las distancias
con `sideLeftOffsetMm` y `sideRightOffsetMm` midiendo separaciones conocidas. La orden `calsonda` sigue
calibrando solo los tres sensores originales; no calibra estos nuevos offsets.

Despues de comprobar medidas y montaje: `set sideEnabled 1`, seguido de `save`. Se conserva la lectura
y el diagnostico de los nuevos aunque el control este desactivado. Si no estan disponibles, el robot
sigue usando los originales; no se exige su presencia para iniciar una mision.

En rectas, dos muestras nuevas consistentes pueden aportar votos de pared/hueco a las aristas aun desconocidas de la celda actual.
Se exige pose precisa, cono alejado de esquinas/postes y lecturas validas: una lectura perdida **no** es
un hueco. Para recentrar con los nuevos, se exigen ambas paredes confirmadas, lecturas recientes a los dos lados y un ancho de pasillo coherente. Se usa la diferencia derecha-izquierda para evitar que un error del ancho aprendido genere dos correcciones opuestas. Con una sola pared o lecturas incompletas, siguen trabajando los diagonales y el giroscopio.
El filtro combina el desplazamiento con encoders y giroscopio; un solo lateral no mide directamente el
angulo. En giros o recuperaciones se suspende esta correccion. Si un nuevo sensor deja de responder,
su dato caduca a los 150 ms y se vuelve al control de los originales; el dato viejo no sigue votando.
Los sensores nuevos no sobrescriben directamente un paso ya confirmado: el observador original y la comprobacion frontal siguen gestionando contradicciones. Los modos 1, 2 y 3 y el control PWM de motores se mantienen.

Pruebas reproducibles (montaje simulado, no validacion fisica del robot):

```bash
cmake -S . -B build
cmake --build build -j
ctest --test-dir build --output-on-failure
scripts/banco.sh 80 --set sideEnabled=1
SIM_SIDES=0 scripts/banco.sh 20 --set sideEnabled=1
SIM_FAIL_SIDE_LEFT=3 SIM_FAIL_SIDE_RIGHT=3 scripts/banco.sh 20 --set sideEnabled=1
```

`--set sideEnabled=1` activa los cinco en el simulador; `SIM_SIDES=0` simula ausencia de los nuevos.
La simulacion usa la geometria configurada tambien como geometria fisica de los nuevos; no comprueba
un montaje real mal medido. Evaluar en el robot rectas, aperturas laterales, giros y desconexion de un
sensor, empezando con el perfil prudente. No se garantiza una mejora de rapidez en todos los mapas.

Validacion del soporte inicial: 408 comprobaciones sin fallos; banco de 81 pistas con errores, cinco
sensores: 81 completadas, 80 sin contacto, 8 contactos y 4 aristas mal clasificadas en total. Referencia
con tres sensores: 81 completadas, 79 sin contacto, 6 contactos y 6 aristas mal clasificadas. La mejora
no es uniforme: algunos indicadores mejoran y otros no; falta medir el montaje real. Las misiones de
prueba con ausencia de los dos nuevos y con perdida de sus lecturas completan exploracion y carrera.
