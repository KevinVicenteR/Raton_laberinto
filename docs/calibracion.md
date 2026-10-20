# Calibración

## Lo que hace solo (no hay que tocar nada)

| Qué | Cuándo |
|---|---|
| Giroscopio en reposo | al arrancar cualquier modo ("No tocar...") |
| Desfase de los tres láseres y ancho real del pasillo | al arrancar el **modo 1**, girando en la salida (unos 3 s). Se guarda solo |
| Láseres laterales y medida de la celda en otra pista | al arrancar el **modo 3** |
| Modelo de los motores (y lo que cambia con la batería) | en cada recta mientras corre; se guarda solo al terminar |
| Escala del giroscopio | con las paredes después de cada giro; se guarda sola al terminar |
| Paso real de la celda | en marcha en el modo 3 |

En la competencia basta con poner el ratón en la salida y arrancar el modo 1. Lo único que el ratón no puede saber solo es **dónde está la meta** (no tiene sensor de suelo para ver la línea blanca) y el tamaño del laberinto y la salida si no son los del reglamento: ver los puntos 1 y 2.

Todas las órdenes se envían por Bluetooth (web o app UART, nombre `Raton`) o por el Monitor Serie a 115200. `save` guarda los parámetros en flash (las calibraciones automáticas ya se guardan solas).

## 0. Cableado (una vez)

Modo 6 "Ruedas", con el ratón **levantado**. Cada línea debe dar:
- `izq adelante`: izq > 0 y der ≈ 0;
- `der adelante`: der > 0 y izq ≈ 0.

Si no es así, cambia `INVERT_*` o `ENCODERS_SWAPPED` en `firmware/config.h`. Comprueba también el giroscopio: en el modo 4, girar el ratón a la izquierda a mano debe subir "Giro". Si baja, cambia `GYRO_SIGN`.

## 1. Pista

El reglamento da medidas aproximadas. Mide con una regla:
- pasillo (de pared a pared) y grosor de pared → `set wallMm 12` y `set cellMm <pasillo + pared>`;
- tamaño en celdas → `set mazeW 19`, `set mazeH 10`;
- salida → `set startX 18`, `set startY 0`, `set startDir 0`.

Coordenadas: (0,0) es la celda de abajo a la izquierda; dirección 0 = norte, 1 = este, 2 = sur, 3 = oeste.

## 2. Meta

El ratón no tiene sensor de suelo y no ve la línea blanca. Se le dice la celda de la meta:
- `G 4,2`, o `set goalX 4` y `set goalY 2`;
- si la meta es una zona, `set goalW 2` y `set goalH 2`.

Recomendación de hardware: un sensor de reflexión (TCRT5000/QRE1113) mirando al suelo permitiría detectar la línea blanca de la meta sin configurarla.

## 3. Sensores (automático)

Cada VL53L0X sale de fábrica con su propio desfase (1–2 cm). Sin corregirlo, el ratón cree estar descentrado y se pega a una pared. En el simulador, con desfases de hasta 16 mm cumple 123 de 161 casos sin calibrar y 159 calibrando (error de la calibración: ~2.5 mm).

- El **modo 1 lo hace solo al arrancar** en la salida (la salida de la competencia tiene pared a los dos lados) y lo guarda. Si la salida está a otra distancia que la de `cellMm`, avisa `PISTA_DISTINTA` y no sale.
- El **modo 3** (práctica) calibra los laterales al arrancar; el frontal lo toma de la última vez del modo 1.
- A mano, solo si se quiere: `calsonda` (o el botón "Calibrar sensores"), `calfront` (solo el frontal, centrado mirando a una pared) y `calside <ancho del pasillo>` (fija `cellMm` y calibra). También se guardan solos.
- `diagMountDeg` (45) y `sensorFovDeg` (25) describen el montaje y el cono; solo cambiarlos si el montaje es otro.

## 4. Motores (automático)

El ratón aprende en cada recta cuánto le falta o le sobra al modelo del motor (`learnMotor 1`): con motores un 40 % más débiles de lo previsto pasa de 56 a 76 de 81 casos en el simulador. Lo aprendido se guarda solo al terminar cada misión. El modo 5 sigue disponible para medir `kV`, `kS` y `motorTauS` desde cero (ratón en el suelo, pasillo recto de 2 celdas), pero ya no hace falta.

## 5. Velocidad

Empieza con los valores por defecto:
- explorar: `searchSpeed 400` (aceleración `accel`/`decel` 3000 mm/s²);
- carrera: `runSpeed 650`, `turnSpeed 280`.

Sube en pasos de 10 % solo si el ratón va limpio. Si en la carrera roza en las curvas, baja `turnSpeed` o usa giros en el sitio. Explorando, los giros son siempre en el sitio: con curvas y paredes aún sin conocer el simulador falla mucho más. En el simulador se puede probar antes: `scripts/run.sh bateria 20 --set searchSpeed=450`.

## 6. Comprobación

- `estado`: tiempo máximo de cálculo de un ciclo (debe quedar muy por debajo de 2000 µs), ciclos tarde y escala del giroscopio aprendida.
- La web muestra la línea `NAV` (sensores, velocidad, posición), el **radar** (los tres conos con su lectura y las paredes de un pasillo centrado como referencia) y las celdas del mapa en directo.
- Tras actualizar el firmware los parámetros guardados vuelven a los de fábrica (cambió la estructura). No hace falta hacer nada: la primera vez que corra el modo 1 se calibra y aprende solo.
