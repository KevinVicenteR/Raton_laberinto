> Actualización 2026-10-22: las suites nativas de núcleo y autonomía ya se pudieron ejecutar y pasan. La web pasa 59 pruebas, build y lint. La compilación ARM sigue fallando porque Windows no ejecuta el toolchain; no se ha cargado la versión en la placa. Véase [calibración completa y PID prudente](calibracion_completa.md). Las limitaciones de ejecución descritas al final documentan la revisión anterior.

# Control, navegación y componentes: revisión experimental

Revisión del 21 de octubre de 2026. Aplicable a `experimental/autonomia-v3`.
Las decisiones siguientes combinan documentación primaria con la inspección del
código; no constituyen una calibración física ya realizada.

## Qué se encontró en el código y qué se cambió

El estimador corregía `gridAngle_` con las paredes, pero el objetivo fijo de
`Motion` conservaba su referencia anterior. Ahora el control recto sigue la
misma referencia corregida, sin reescribir un pivote activo. La corrección de
cuadrícula tiene una velocidad limitada y los cambios de tramo borran la
corrección lateral del tramo anterior.

La derivada de rumbo diferenciaba directamente el error a cada ciclo de 2 ms.
Los saltos de fusión IMU/encoder podían generar cambios rápidos de torque.
Se filtra esa derivada con un IIR de primer orden: tau=25 ms en recta y 12 ms
en giro. Son valores experimentales de esta implementación, no parámetros
prescritos por Arduino. La integral deja de acumularse cuando la saturación
se opone al error y puede descargarse al cambiar este de signo.

El centrado mantiene la cascada de posición lateral hacia rumbo, con zona
muerta de 1,5 mm, corrección máxima de 3,5 grados, filtro de 60 ms y cambio
limitado a 20 grados/s. La relocalización por innovaciones rechazadas exige
cuatro hipótesis de posición coherentes: cuatro reflejos alternantes ya no
bastan para mover el estimador. La parada frontal sigue usando datos recientes;
no se ha colocado una media lenta delante de la detección de obstáculos.

La celda estimada no puede avanzar fuera del tamaño configurado del laberinto;
una contradicción activa recuperación. Antes de salir desde una celda cuyo
mapa ya se conoce se adquieren 160 ms de observaciones nuevas. Tener paredes
cargadas no equivale a haber medido de nuevo la posición del robot.

En la web había otros problemas independientes: reconstruir la escena Three.js
al cambiar cada paquete CELL reiniciaba la cámara; además, perder P/H/G podía
cambiar el origen absoluto al relativo. Ahora se actualizan las paredes en
la misma escena y NAV conserva su marco absoluto. Se normaliza el paso de
cada muestra y se cortan líneas en saltos o desconexiones. Las poses exteriores
se avisan y conservan en el registro, sin recortarlas para aparentar precisión.
Los ecos aproximados del ToF se muestran solo al activar su opción y ya no
se proyecta una lectura del cono como tres obstáculos independientes.

## Algoritmos investigados y decisión

| Técnica | Para qué ayuda | Aplicación en este ratón |
| --- | --- | --- |
| Mediana de ventana corta | Rechazar picos aislados | Candidata para el canal de estimación. Hay que modelar su demora; no aplicada a ciegas a la parada frontal. |
| IIR de primer orden y límite de pendiente | Reducir sacudidas por ruido | Aplicados a la derivada y a la corrección lateral. |
| Kalman con puerta de innovación | Fusionar odometría y paredes con incertidumbre | Se conserva el filtro existente y se exige coherencia antes de relocalizar por rechazo reiterado. |
| PID con integración condicional y feedforward | Evitar acumulación con motores saturados | Se conserva kS/kV/kA y se añade integración condicionada en operación autónoma. |
| Pure Pursuit | Seguir un punto adelantado de una trayectoria conocida | Alternativa para una fase posterior de rutas métricas. Su seguimiento geométrico no soluciona por sí solo una pose errónea o un cruce aún desconocido. |

Fuentes primarias: [mediana, WPILib](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/filters/median-filter.html),
[limitador de pendiente, WPILib](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/filters/slew-rate-limiter.html),
[PID/anti-windup, ROS control_toolbox](https://control.ros.org/rolling/doc/api/classcontrol__toolbox_1_1Pid.html),
[feedforward, WPILib](https://docs.wpilib.org/en/stable/docs/software/advanced-control/controllers/feedforward.html),
[Coulter, Pure Pursuit, CMU-RI-TR-92-01](https://publications.ri.cmu.edu/implementation-of-the-pure-pursuit-path-tracking-algorithm).

## Calibración del control: orden de trabajo

1. Verificar polaridades y unidades del encoder por vuelta real de rueda;
   confirmar diámetro efectivo bajo carga y separación efectiva entre ruedas.
   Una cifra de encoder del eje del motor no equivale automáticamente a cuentas
   decodificadas por vuelta de la rueda tras la reductora.
2. Medir el sesgo de giro en reposo, con motores sin vibrar. Verificar giros
   reales de 90/180 grados antes de cambiar ganancias del controlador.
3. Caracterizar cada motor y sentido: rozamiento de arranque, velocidad frente
   a duty y respuesta transitoria. Estimar el modelo de avance antes de pedir
   que la integral compense todo el desbalance o la batería.
4. Ajustar primero el lazo de rumbo y el de avance, por separado, con integral
   pequeña o desactivada durante la caracterización. Medir sobreimpulso,
   asentamiento, error y fracción de tiempo saturado. Añadir solo la integral
   necesaria para sesgos persistentes y validar su descarga.
5. Ajustar después el centrado de pasillo; no modificar simultáneamente PID,
   escala de giro, geometría de sensores y ganancias laterales.
6. Validar a varias velocidades, al cambiar el estado de batería y en cruces,
   esquinas, entradas dobles y recuperación. Comparar contacto, error lateral,
   rumbo y tiempo; una carrera aislada exitosa no prueba calibración general.

`CONTROL` registra vref/v en mm/s, aref/a/omega en décimas de grado o de
grado/s, uL/uR en milésimas de duty y dt en microsegundos. Su publicación cada
500 ms permite revisar tendencias, no identificar por sí sola vibraciones
rápidas. Para sintonizar el ancho de banda físico se necesita una captura local
más rápida o instrumentación del robot. No se publican ganancias "óptimas"
sin esos datos. Los valores del simulador son una comprobación de regresión.

WPILib advierte que las ganancias obtenidas por identificación son un punto
inicial y que los filtros introducen demora que hay que considerar:
[análisis y retardos de medición](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/analyzing-gains.html).

## ¿Puede la Nano ejecutar procedimientos separados?

La Nano 33 BLE Sense Rev2 usa nRF52840 y el core Arduino basado en Mbed OS.
El SoC tiene un Cortex-M4F a 64 MHz, 256 KB de RAM y 1 MB de flash. Puede
planificar tareas/hilos, eventos e interrupciones; no tiene dos núcleos para
asignar físicamente sensores a uno y motores al otro. Los hilos ordinarios
comparten memoria: separación de responsabilidades no equivale a aislamiento
de fallos entre procesos.

Propuesta de distribución, aún pendiente de implementar y medir en placa:

| Trabajo | Organización propuesta | Regla de acceso |
| --- | --- | --- |
| Encoders | Interrupciones breves que cuentan flancos | Copia atómica de contadores; sin BLE ni I2C dentro de la ISR. |
| Pose y PID | Propietario único del estado; prioridad máxima de aplicación, período objetivo de 2 ms | Usar marcas de tiempo reales; no escribir flash ni esperar a la web. |
| Sensores | Lectura no bloqueante por turnos; cada bus con un propietario | VL53L0X en Wire externo; BMI270 en Wire1 interno, como en el firmware actual. |
| Planificación y recuperación | Estado incremental y presupuesto acotado por ciclo | Recibe una instantánea coherente; entrega objetivos al control. |
| BLE/telemetría | Cola limitada y servicio de menor prioridad | Una sola tarea dueña de ArduinoBLE; descartar telemetría antigua antes que retrasar control. |
| Persistencia | Trabajo diferido con motores quietos | No guardar flash durante una maniobra. |

Actualmente se conserva la arquitectura de control y su servicio cooperativo;
esta entrega no afirma haber migrado a hilos RTOS. La migración exige medir
latencias, consumo de pila y comportamiento de la librería BLE en el core
instalado. Arm retiró el soporte activo de Mbed en julio de 2026; conviene
mantener fija la versión del core Arduino validada antes de otra migración.

Fuentes: [Arduino Rev2](https://docs.arduino.cc/hardware/nano-33-ble-sense-rev2/),
[Nordic nRF52840](https://www.nordicsemi.com/Products/nRF52840),
[core basado en Mbed](https://blog.arduino.cc/2019/07/31/why-we-chose-to-build-the-arduino-nano-33-ble-core-on-mbed-os/),
[estado de Mbed, Arm](https://github.com/ARMmbed).
La API del hilo, sus prioridades y memoria de pila se pueden contrastar en el
[código de Thread de Arm](https://github.com/ARMmbed/mbed-os/blob/master/rtos/include/rtos/Thread.h).

## Componentes y uso indicado por fabricantes

La valoración siguiente es de adecuación al diseño, no una medición de la
calidad de los módulos físicos de Kevin.

| Componente | Valoración para el laberinto | Uso/calibración y límite relevante |
| --- | --- | --- |
| Nano 33 BLE Sense Rev2 | Capacidad razonable para control y mapa por celdas; comprobar la carga real del lazo | Usar el core y la librería de IMU correspondientes a Rev2. Mantener tensión lógica y buses según el esquema de la placa. |
| Cinco VL53L0X | Útiles para distancias a paredes; no son escáneres 3D | Inicializar direcciones por XSHUT. Cada uno necesita su compensación de montaje. El cono puede incluir postes, piso y varias superficies. |
| BMI270 | Adecuado para velocidad de giro; integrar su señal acumula deriva | Bosch distingue compensación de offset y de sensibilidad. Las funciones internas tienen requisitos y no equivalen al sesgo software actual. |
| DRV8833 | Adecuado si corriente, tensión y disipación del conjunto lo permiten | TI especifica 2,7–10,8 V y corrientes dependientes del encapsulado/disipación; 2 A de pico no significa 2 A continuos garantizados por cualquier módulo. Revisar nFAULT y alimentación. |
| N20 con encoder | La reductora y la adherencia pueden limitar precisión aun con encoder | N20 es un formato, no identifica fabricante, relación, corriente de bloqueo ni resolución. Falta el modelo exacto para contrastar su hoja de datos. |

ST recomienda para offset del VL53L0X un blanco blanco de 88 % a 100 mm en
ambiente oscuro, con SPAD y referencia ya calibrados. El resultado debe
guardarse en el host. Para crosstalk, primero offset y luego el procedimiento
con el conjunto óptico final, distancia seleccionada y blanco gris del 17 %.
La API completa permite ambas operaciones; los offsets software del proyecto
y la librería ligera usada no equivalen a ejecutar esa calibración completa.
Un presupuesto de tiempo mayor puede mejorar precisión, a costa de latencia.

Bosch describe offsets y compensación automática en uso con condiciones de
activación concretas; no deben escribirse a la vez los mismos offsets por
software y por una función automática habilitada sin revisar su interacción.
En el proyecto se mantienen el sesgo software en reposo y la escala aprendida;
no se activa silenciosamente una segunda calibración interna.

TI describe recuperación por sobrecorriente, temperatura y subtensión. Un
atasco con corriente inferior al umbral puede no activar nFAULT; se mantiene
la detección de falta de progreso mediante sensores y encoder. El driver no
tiene una calibración PID propia: el PID pertenece al microcontrolador.

Fuentes: [ST UM2039, secciones 2.5 y 2.6](https://www.st.com/resource/en/user_manual/um2039-world-smallest-timeofflight-ranging-and-gesture-detection-sensor-application-programming-interface-stmicroelectronics.pdf),
[librería Pololu y timing budget](https://github.com/pololu/vl53l0x-arduino/blob/master/README.md),
[Bosch BMI270, secciones 4.12 y 4.13](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi270-ds000.pdf),
[TI DRV8833, corriente y protecciones](https://www.ti.com/lit/ds/symlink/drv8833.pdf),
[Arduino: librerías Rev2](https://support.arduino.cc/hc/en-us/articles/11729186296476-Use-the-new-sensor-libraries-for-Nano-33-BLE-Rev2-and-Nano-BLE-Sense-Rev2).

No se cambian XSHUT A0/A1/A2/A6/A3 ni SDA A4/SCL A5. Faltan mediciones
físicas para verificar óptica de las carcasas, vibración, alimentación y
corriente de bloqueo; no se atribuye todo temblor al software ni se promete
posición exacta solo por disponer de estos sensores.

## Evidencia y pendientes de esta revisión

La compilación CMake del núcleo, simulador y firmware con stubs es válida;
no sustituye compilar y cargar el binario ARM. La prueba sintética de ruido
de rumbo redujo el RMS de los cambios de mando diferencial de 0,063875 a
0,006266. Es una comparación reproducible de ruido inyectado, no una medición
de vibración o reducción de colisiones en la Nano física.

Las misiones T/cruz, el arranque degradado y la recuperación persistente
conservaron su comportamiento en las ejecuciones intermedias. La verificación
de diagonal expuso que una ruta explorada no tiene por qué elegir la misma
diagonal tras cambiar el controlador; se añadió una prueba de ruta conocida
y adquisición de paredes antes de salir. La repetición final debe confirmar
esta ruta, el filtro robusto y el pasillo con ruido y asimetría añadidos.

Windows Control de aplicaciones bloqueó el último ejecutable nativo recién
compilado. Por ello no se declara pasada la suite nativa final ni calibrado
el hardware. La captura y el Markdown adjuntos no suministraron contenido
legible mediante la lectura disponible y la descarga falló por permisos del
entorno. No se han inferido mediciones de esos archivos; el análisis se basa
en el código, pruebas ejecutadas y fuentes primarias enlazadas.

La versión final de la web pasa 56 pruebas, TypeScript/build y lint. Las nuevas
regresiones cubren el origen sin P/H/G, el paso aprendido, saltos de trayectoria,
poses fuera de límites, ecos durante giros y CELL repetido sin reconstruir
paredes. La integración conserva los tres modos y la recuperación persistente;
la migración RTOS es una propuesta de diseño, no una función ya implementada.
