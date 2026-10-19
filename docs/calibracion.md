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

- **Frontal:** ratón en el **centro** de una celda mirando a una pared → `calfront` (fija `frontStopMm`).
- **Laterales:** ratón centrado y recto en un pasillo con pared a los dos lados → `calside <ancho del pasillo en mm>`. Fija el ángulo real de los laterales y el tamaño de celda.
- **Offsets:** si un sensor lee distinto de la regla, `set leftOffsetMm <real - lectura>` (también `rightOffsetMm` y `frontOffsetMm`).

## 4. Motores

Modo 5, con el ratón en el suelo al principio de un pasillo recto de al menos 2 celdas. Mide `kV`, `kS` y `motorTauS` y los aplica. Repite 2–3 veces, mira que salgan parecidos y haz `save`.

## 5. Velocidad

Empieza con los valores por defecto:
- explorar: `searchSpeed 320`;
- carrera: `runSpeed 650`, `turnSpeed 280`.

Sube en pasos de 10 % solo si el ratón va limpio. Si en la carrera roza en las curvas, baja `turnSpeed` o usa giros en el sitio. Para el bloque de explorar se puede probar `searchSpeed 380–420`: en el simulador llega antes pero con algún roce. En el simulador se puede probar antes: `scripts/run.sh bateria 20 --set searchSpeed=380`.

## 6. Comprobación

- `estado`: tiempo máximo de cálculo de un ciclo (debe quedar muy por debajo de 2000 µs), ciclos tarde y escala del giroscopio aprendida.
- La web muestra la línea `NAV` (sensores, velocidad, posición) y las celdas del mapa en directo.
