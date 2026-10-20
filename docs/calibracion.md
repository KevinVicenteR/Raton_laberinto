# Calibración en la pista (hacer antes de competir)

Todas las órdenes se envían por Bluetooth (web o app UART, nombre `Raton`) o por el Monitor Serie a
115200. Tras ajustar, `save` guarda los parámetros en flash.

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

## 3. Sensores

Cada VL53L0X sale de fábrica con su propio desfase (1–2 cm). Sin calibrar, el ratón cree estar descentrado y se pega a una pared. En el simulador es lo que más cuenta: con desfases de hasta 16 mm cumple 123 de 161 casos sin calibrar y 158 calibrando.

- **`calsonda`** (o el botón "Calibrar sensores" de la web): ratón en el suelo, en el **centro** de una celda de la competencia con pared a los dos lados. Gira en el sitio: mira cada pared de frente con el frontal y pone cada lateral perpendicular a su pared. Corrige `frontOffsetMm`, `leftOffsetMm` y `rightOffsetMm`. Después `save`.
- El **modo 1 lo hace solo al arrancar** en la salida (la salida de la competencia tiene pared a los dos lados). Si la salida está a otra distancia que la de `cellMm`, avisa `PISTA_DISTINTA` y no sale.
- El **modo 3** (práctica) también calibra los laterales al arrancar, pero el frontal tiene que venir calibrado de antes (`calsonda` en la pista de la competencia + `save`).
- `calfront`: solo el frontal (centrado en una celda, mirando a una pared).
- `calside <ancho del pasillo>`: fija `cellMm` = pasillo + pared y hace `calsonda`.
- `diagMountDeg` (45) y `sensorFovDeg` (25) describen el montaje y el cono; solo cambiarlos si el montaje es otro.

## 4. Motores

Modo 5, con el ratón en el suelo al principio de un pasillo recto de al menos 2 celdas. Mide `kV`, `kS` y `motorTauS` y los aplica. Repite 2–3 veces, mira que salgan parecidos y haz `save`.

## 5. Velocidad

Empieza con los valores por defecto:
- explorar: `searchSpeed 400` (aceleración `accel`/`decel` 3000 mm/s²);
- carrera: `runSpeed 650`, `turnSpeed 280`.

Sube en pasos de 10 % solo si el ratón va limpio. Si en la carrera roza en las curvas, baja `turnSpeed` o usa giros en el sitio. Explorando, los giros son siempre en el sitio: con curvas y paredes aún sin conocer el simulador falla mucho más. En el simulador se puede probar antes: `scripts/run.sh bateria 20 --set searchSpeed=450`.

## 6. Comprobación

- `estado`: tiempo máximo de cálculo de un ciclo (debe quedar muy por debajo de 2000 µs), ciclos tarde y escala del giroscopio aprendida.
- La web muestra la línea `NAV` (sensores, velocidad, posición), el **radar** (los tres conos con su lectura y las paredes de un pasillo centrado como referencia) y las celdas del mapa en directo.
- Tras actualizar el firmware los parámetros guardados vuelven a los de fábrica (cambió la estructura): repite modo 5 y `calsonda`, y `save`.
