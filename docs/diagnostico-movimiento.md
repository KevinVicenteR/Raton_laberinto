# Auditoría de movimiento — 17 de octubre de 2026

## Registros reales (sin mover la placa)

Reproducción: `.venv/bin/python -m raton_sim.log_audit`. Se comprueba CRC-8 y se recuperan las
tramas completas incluso si llegaron concatenadas. Los resúmenes por archivo están en
`simulator/results/motion-audit/real.json`. No se mezclan pruebas de ruedas levantadas con navegación.

| Registro | Muestras en marcha | Rumbo pico a pico | Lateral aparente pico a pico | Velocidad en marcha | DECIDE → siguiente movimiento | Error final de giro medio / máximo |
|---|---:|---:|---:|---:|---:|---:|
| entrenar-c192-1909 | 25 | 1° | 37.86 mm | 261.2 mm/s | 2.16 s | 1.65° / 3.68° |
| modo3-1833 | 19 | 15° | 5.67 mm | 212.5 mm/s | 1.62 s | 2.18° / 4.69° |
| modo3-auto-1902 | 17 | 9° | 50.22 mm | 293.9 mm/s | 0.385 s | 1.88° / 3.07° |
| modo3-c260-1844 | 21 | 18° | 19.57 mm | 288.0 mm/s | 1.26 s | 1.98° / 4.41° |

El lateral aparente es `(I-D)*sin(31°)/2`, solo con ambos valores válidos. No es una medida externa:
mezcla desplazamiento, ángulo y esquinas vistas por los rayos diagonales. Rumbo: mediana de los
rangos en rectas de ≥5 muestras, ≥0.5 s y sin huecos de recepción >1 s. En modo3-1813 se ve un
periodo aparente de 6.55 s; en los demás no hay ciclos suficientes para estimar un periodo fiable.
`v` es velocidad por encoders y la cifra es una media de muestras **en marcha**, no velocidad media
física de la misión. No existe referencia externa para separar patinaje o calibración de ruedas.
Los tiempos son de recepción Bluetooth: no equivalen al reloj de control, especialmente con drops.
En c192 se descartaron 46 candidatos con CRC incorrecto y quedaron 408 tramas válidas.

Las líneas MOTION solo dan error final: no permiten medir el máximo sobrepaso de todos los giros
ni contar cada reintento interno. OBS a 2 Hz tampoco resuelve los empujones de 5 ms. El estado
PARADO muestra normalmente 0° de cambio entre muestras, pero las pruebas de ruedas incluyen hasta
4°: no demuestra oscilación mecánica espontánea. El bailoteo bajo estado GIRO sí es coherente con
el mínimo PWM que se reimpone al caer la tasa por debajo de 20°/s. Estas son limitaciones de evidencia,
no ceros de oscilación inventados.

## Causas y tiempos del código anterior

- `resetHeadingToGrid` invertía el signo del error inicial: un giro residual positivo se corregía
  inicialmente en el sentido equivocado. Se conserva ahora el signo de `gridError()`.
- Cascada lateral con KP_WALL 0.185, objetivo filtrado con alfa 0.05 (≈98 ms de retardo a 200 Hz)
  y amortiguación de yaw 0.0006. La tasa lateral del Kalman no participaba en esta cascada.
- El PI de velocidad no podía bajar de PWM_MIN_RUN, incluso cuando excedía la consigna. Eso explica
  que velocidades declaradas de exploración bajas coexistieran con muestras de 260–294 mm/s.
- Giro PD con mínimo 0.28 al parar, incluso cerca del objetivo; tolerancia 3°, aceptación 5° y
  fallo solo a 10°. Podía repetir tres intentos de hasta 3 s. Precisión y aceptación eran diferentes.
- El mapa aprendido solo vivía en RAM; `mapTrack` podía anunciar completo tras descartar fronteras.

Desglose nominal con una entrega VL53L0X cada 30 ms (estimación estática, no tiempos medidos por
Bluetooth). Hay llamadas condicionales: no se deben sumar todas en cada cruce.

| Operación | Antes | Ahora |
|---|---:|---:|
| Inicio de runCells | 8 entregas ≈240 ms **pero timeout real de waitFresh=200 ms** | 2 entregas ≈60 ms solo tras giro, filtro invalidado |
| stableFrontMm | 9 entregas ≈270 ms, mínimo 3 válidas | 5 ≈150 ms, mínimo 3 válidas |
| calibrateStill | 15 laterales + 9 frontales ≈720 ms | 7 + 5 ≈360 ms |
| senseStill | 2 calibraciones + alignToFront (otra mediana): ≈1710 ms sin movimiento | una calibración; repetir solo si hace falta mover para centrar |
| senseTurning | 3 giros (90, −180, 90) + 3 medianas ≈810 ms, además de giros/paradas | mismos giros solo si no hubo aproximación, medianas ≈450 ms |
| stopMotors | delay(60) sin servicio + SETTLE 40 =100 ms, luego rueda libre | 20+15=35 ms atendiendo sensores/órdenes; mantener freno |
| applyRun → frontWallStopped | senseCell: 5 entregas ≈150 ms tras cada tramo | reutilizar las dos entregas frontales confirmadas en marcha |
| settled de giro | 3 ciclos y tasa <40°/s; aceptación parada a 5° | 3 ciclos y tasa <8°/s; error final <3°, máximo 2 intentos |

`Gyro::calibrate` sigue al arrancar una misión, no en cada celda. Las medianas y esperas nuevas
atienden IMU, odometría, telemetría y órdenes. El frontal se confirma por su propio contador de
entregas, no por una actualización de cualquiera de los tres láseres.

## Comparación reproducible de simulación

Comandos desde la raíz:

```sh
.venv/bin/python -m raton_sim.motion_audit /tmp/before.json --legacy
.venv/bin/python -m raton_sim.motion_audit /tmp/after.json
.venv/bin/python -m raton_sim.motion_audit /tmp/before-margins.json --legacy --margins
.venv/bin/python -m raton_sim.motion_audit /tmp/after-margins.json --margins
```

Ambos controladores usan la misma planta angular: fricción PWM_MIN_RUN, 900°/s por PWM útil,
constante de tiempo 50 ms, aceleración 900°/s² y frenado 1800°/s². Son **supuestos de N20**, no una
identificación experimental. `--legacy` reconstruye el PD, mínimo, esperas y parámetros anteriores;
la lógica de navegación y la planta son comunes. Se archiva también `original-ideal.json`, obtenido
antes de editar: el simulador anterior imponía 180°/s y carecía de inercia, por lo que no podía
validar oscilaciones del firmware. No se debe atribuir toda la diferencia entre ambos modelos al robot.

La banda nominal usa el ruido, dropout y errores de montaje predeterminados; semilla sensorial 1,
competencia 19×10 con meta (4,2), aleatorios 100/101/102, modo 3 **con meta** (no mapeo completo).
Todas las métricas de cada celda de la tabla son antes → después. Amplitudes: promedio de rangos
pico a pico en tramos rectos de ≥0.5 s. Pausa: adquisición sin maniobra más parada, por tramo/decisión;
excluye el tiempo empleado girando. Velocidad: distancia total / tiempo total. Sobrepaso: máximo
exceso firmado respecto a la consigna durante cada giro, promediado entre giros. Las oscilaciones de
ruido alrededor de cero no implican un ciclo límite sostenido.

| Pasillo / caso | Rumbo pp ° | Lateral pp mm | Pausa s | Media mm/s | Sobrepaso ° | Meta s | Contactos |
|---|---:|---:|---:|---:|---:|---:|---:|
| 192 / competencia4 | 4.92 → 0.22 | 25.03 → 0.79 | 0.301 → 0.117 | 83.3 → 168.0 | 0.00 → 0.12 | 77.5 → 38.4 | 0 → 0 |
| 192 / competencia3 | 2.96 → 0.29 | 9.93 → 0.38 | 0.192 → 0.057 | 64.0 → 120.9 | 0.00 → 0.11 | 221.6 → 117.7 | 0 → 0 |
| 192 / aleatorio0 | 3.03 → 0.27 | 9.88 → 0.31 | 0.200 → 0.059 | 62.9 → 120.8 | 0.00 → 0.12 | 168.0 → 87.5 | 0 → 0 |
| 192 / aleatorio1 | 3.11 → 0.28 | 10.55 → 0.40 | 0.198 → 0.059 | 61.9 → 118.9 | 0.00 → 0.11 | 289.2 → 151.2 | 0 → 0 |
| 192 / aleatorio2 | 2.75 → 0.30 | 8.61 → 0.39 | 0.189 → 0.057 | 64.9 → 122.9 | 0.00 → 0.14 | 181.3 → 95.9 | 0 → 0 |
| 168 / competencia4 | 4.98 → 0.21 | 22.13 → 0.70 | 0.301 → 0.117 | 76.4 → 158.7 | 0.00 → 0.09 | 74.5 → 35.9 | 0 → 0 |
| 168 / competencia3 | 3.07 → 0.32 | 8.73 → 0.37 | 0.195 → 0.058 | 60.0 → 115.6 | 0.00 → 0.12 | 208.6 → 108.5 | 0 → 0 |
| 168 / aleatorio0 | 3.09 → 0.29 | 8.63 → 0.32 | 0.197 → 0.060 | 58.7 → 115.6 | 0.00 → 0.12 | 158.8 → 80.8 | 0 → 0 |
| 168 / aleatorio1 | 3.15 → 0.33 | 9.18 → 0.39 | 0.199 → 0.060 | 57.8 → 113.8 | 0.00 → 0.12 | 273.4 → 139.1 | 0 → 0 |
| 168 / aleatorio2 | 2.84 → 0.32 | 7.59 → 0.35 | 0.187 → 0.056 | 60.7 → 117.7 | 0.00 → 0.14 | 171.1 → 88.5 | 0 → 0 |

El sobrepaso nominal pasa de 0 a aproximadamente 0.1°: **no mejora esa métrica**. La precisión final
se exige a <3° y se prueba también en giros ±5°, ±90° y 180°, ganancia de motor 0.75/1/1.2 y
constante de tiempo 80/50/30 ms. El simulador anterior se afinaba artificialmente hasta 0.5°;
el camino profiled_turn ya usa la tolerancia del firmware, sin ese refinamiento extra.

| Pasillo / caso | Periodo de cruces de rumbo, s (antes → después) |
|---|---:|
| 192 / competencia4 | no estimable → 0.68 |
| 192 / competencia3 | 1.11 → 0.64 |
| 192 / aleatorio0 | 0.24 → 0.53 |
| 192 / aleatorio1 | 0.97 → 0.59 |
| 192 / aleatorio2 | no estimable → 0.50 |
| 168 / competencia4 | no estimable → 1.57 |
| 168 / competencia3 | no estimable → 0.50 |
| 168 / aleatorio0 | no estimable → 0.49 |
| 168 / aleatorio1 | 0.93 → 0.59 |
| 168 / aleatorio2 | no estimable → 0.64 |

### Ensayo adverso: no aprobado

`--margins` combina salida 12 mm descentrada, 2° de error inicial, deriva 3°/s, ruido 6 mm,
8 % de dropout, motor al 75 % y tau=80 ms. Resultado: **10 → 11 contactos; 7/10 → 3/10 llegadas**.
Hay una regresión; las velocidades propuestas no están calificadas para esos márgenes. Los fallos
son giros bloqueados/PEGADO por posición desplazada, aunque la oscilación de rumbo disminuye.
Los JSON completos conservan cada fallo y contacto. No se acepta este resultado como validación
robusta ni como autorización para usar estos parámetros en la placa. Hace falta corregir la
localización/centrado bajo perturbaciones y repetir el banco; una planta identificada con encoders
e IMU de mayor frecuencia también es necesaria para afirmar mejoras reales de velocidad o sobrepaso.

## Persistencia y alcance

Modo 6: explorar una rejilla 40×40 desde (20,20), volver, recuperar el rumbo y guardar en TDBStore.
Modo 3 con meta: cargar mapa y geometría con versión/CRC antes de calcular la ruta.
ERASE_MAP borra tanto RAM como flash cuando está parado. El modelo Python puede usar
`Case.learned_path` para conservar el mismo formato binario entre instancias; su prueba crea una
pista, regresa, guarda, inicia un simulador nuevo y recorre una ruta cargada sin contactos.
C++ prueba ida/vuelta, todas las alteraciones de un bit del registro, truncado y rechazo sin mutación;
una serpiente de 1600 celdas comprueba el cambio de distancias de inundación a 16 bits.

Los límites de memoria se informan explícitamente: una pista arbitrariamente grande no cabe en un
microcontrolador finito. No se guardan aprendizajes incompletos ni se marcan fronteras fallidas como
vistas. La supervivencia ante apagado está implementada con TDBStore transaccional; no se ha probado
físicamente un corte de alimentación porque no se ha subido firmware ni se ha escrito a la placa.
