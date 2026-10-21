# Autonomía v3 experimental

Rama `experimental/autonomia-v3`, basada en `ea7bd38`. No se fusiona en
`arquitectura_v2` ni se modifica la copia local de `sin_arrancador`.

## Los tres modos de placa y web

| Orden | Modo | Comportamiento |
|---|---|---|
| `1` / `s1` | Autónomo completo | Diagnóstico, búsqueda de meta, mapeo de todas las ramas alcanzables, regreso, guardado y resolución aprendida. |
| `2` / `s2` | Competencia | Busca la meta sin exigir explorar todas las ramas; conserva el mapa persistido anterior. |
| `3` / `s3` | Camino más corto | Recalcula la ruta mínima en celdas sobre aristas confirmadas del mapa guardado. Sin mapa/ruta permanece parado. |

`m1`–`m3` seleccionan; `x` detiene. Elegir modo, conectar y resincronizar nunca
arrancan motores. El arranque requiere una orden o el gesto físico existente.
El modo autónomo utiliza una sola inicialización de pose y un plazo compartido
de 595 s para todas sus fases; no recoloca virtualmente el robot entre ellas.
La posición de salida, dimensiones y meta siguen siendo parámetros conocidos.

## Hardware conservado

Nano 33 BLE Sense Rev2 (BMI270), cinco VL53L0X y DRV8833/N20 con encoders.
DI/F/DD: XSHUT A0/A1/A2; LI/LD: A6/A3; SDA A4 y SCL A5. Direcciones
0x30–0x34. No se cambian polaridades ni pines.

Los cinco canales se muestran y diagnostican. La fusión lateral continúa
condicionada por `sideEnabled`: no se deducen offsets o posiciones de montaje
que no se han medido. Los offsets absolutos necesitan referencias conocidas.

## Las ocho mejoras

1. **Búsqueda estable:** mantiene una frontera alcanzable como objetivo hasta
   observarla o bloquearse; no borra la salida opuesta en T o cruces.
2. **Centrado por confianza:** conserva la fusión de sonda y laterales; pondera
   el mando por incertidumbre y limita su variación. Excluye lecturas antiguas
   o geométricamente incompatibles.
3. **Velocidad adaptativa:** limita lo desconocido a 220 mm/s y baja a 150
   ante incertidumbre o sensores degradados; 120 tras recuperación. Se respetan
   los topes del perfil físico y del motor. La detección de oscilación reduce
   temporalmente ganancia y velocidad, sin guardar ese ajuste provisional.
4. **Diagnóstico continuo:** valida reposo y presencia de muestras al arrancar;
   vigila IMU, sentido de encoders, antigüedad y lecturas constantes que
   contradicen una predicción cambiante con paredes confirmadas. Una distancia
   constante junto a una pared no constituye por sí sola una avería.
5. **Recuperación fluida:** añade un recentrado continuo únicamente con chasis
   medido, pasillo conocido y espacio suficiente. Conserva el rescate acotado
   anterior donde esa maniobra no cabe. No aplica una segunda maniobra sobre
   una trayectoria continua interrumpida como si no se hubiera movido.
6. **Curvas durante exploración:** únicamente en celdas visitadas, pasos
   confirmados, incertidumbre reducida y envolvente de chasis comprobada.
   Suspende temporalmente votos del mapa tras la curva.
7. **Diagonales aprendidas:** un tramo a 45° entre puntos medios de dos bordes,
   con cuatro celdas conocidas y abiertas. Comprueba la envolvente del chasis,
   paredes, postes y margen de incertidumbre. No atraviesa un vértice como si
   los postes no existieran. Utiliza entrada/salida conservadoras con parada.
8. **Web y trazabilidad:** tres modos, mapa aprendido 2D/3D, cinco curvas
   temporales sin suavizado y estado de pose/salud/decisiones. Los huecos de
   telemetría interrumpen las curvas. El cono dibujado es esquemático; el VL53L0X
   no entrega una imagen ni un contorno exacto del obstáculo.

Las trayectorias avanzadas requieren `autoGeometryConfirmed=1`, después de
medir y configurar el cuerpo, ruedas y sensores. Cambiar geometría invalida
esa confirmación; no activa por sí sola la fusión lateral. En un pasillo de
155 mm pueden quedar descartadas por falta de margen: esto es deliberado.

## Posición y avance real

`NAV` publica posición estimada en milímetros. `POSE` añade incertidumbre
longitudinal/lateral, visitas estimadas, contraste externo de avance, indicios
de colisión y oscilaciones detectadas. Las visitas del planificador siguen
procediendo de la localización estimada; `confirmed` es una evidencia adicional,
no una garantía de ubicación exacta ni un reemplazo de las visitas persistidas.

Se contrasta el desplazamiento de encoders con el acercamiento a una pared
que la sonda identifica. Si las ruedas cuentan avance y la distancia no cambia,
se solicita rescate por posible patinaje/colisión. No se declara avance externo
confirmado donde la geometría no proporciona esa referencia. Se mantienen los
detectores previos de seguimiento, pared inesperada y giros incompletos.

## Qué puede autorreparar

Un fallo de transporte láser permite hasta tres rondas de recuperación por
misión: parar motores, esperar reposo, reiniciar solo el módulo afectado por
XSHUT, reasignar dirección y exigir varias muestras nuevas y estabilidad.
Si hay otro dispositivo en 0x29 se rechaza la reasignación para no corromper
el bus. No se reinicia la IMU ni se adivinan polaridades.

Para reanudar tras perder una referencia crítica se exige conservar rumbo y
posición suficientemente acotados y se vuelve al centro de la celda antes de
replanificar. Un lateral original ausente puede permitir continuar con los
restantes; las referencias críticas ausentes, un encoder invertido, una IMU
no válida o un bloqueo persistente terminan la misión. Persistencia significa
intentar recuperaciones justificadas y conservar datos, no conducir sin
referencias ni prometer completar cualquier laberinto con hardware averiado.

El mapa previo solo se sustituye tras mapeo completo y regreso verificado.
Un fallo al guardar impide anunciar éxito. El aprendizaje provisional se
descarta cuando la misión no completa sus condiciones.

## Bluetooth y recuperación de información

La primera vinculación requiere una acción del usuario en Chrome/Edge sobre
HTTPS o localhost. Después la web usa `getDevices()` para el robot previamente
autorizado, con reintentos, cancelación y reconexión. No selecciona al azar
entre varios robots. Desconectar manualmente desactiva la reconexión hasta
volver a vincular. Conectar no manda órdenes de movimiento.

Al enlazar, `sync` funciona incluso durante la misión. La placa envía, por
etapas y con contrapresión: versión, modo, parámetros reales, instantánea del
mapa en RAM, estado actual y eventos pendientes. Las celdas recibidas en vivo
tienen prioridad sobre una instantánea anterior. La web conserva la trayectoria
que recibió; no inventa la trayectoria perdida mientras estuvo desconectada.

El diario mantiene los últimos **32 eventos en RAM**; informa cuántos antiguos
fueron sustituidos. No conserva todas las muestras desconectadas ni sobrevive
a cortar alimentación. El mapa y calibraciones válidas mantienen la persistencia
flash existente. No hay escrituras flash por muestra ni por evento.

Documentación de Web Bluetooth:
https://developer.chrome.com/docs/capabilities/bluetooth

## Validación y límites de esta entrega

Pruebas nuevas: políticas de incertidumbre/velocidad, congelación con evidencia
geométrica, ruido frente a oscilación, patinaje frente a avance contrastado,
diario circular, misiones T/cruz, parada por IMU, transporte frontal recuperable
y permanente, y recorrido diagonal físico simulado con margen suficiente.

En PC se compilan núcleo, simulador y firmware con stubs. Antes de los últimos
cambios pasaron las pruebas existentes y las misiones experimentales de T/cruz
y diagonal; la última ejecución nativa quedó bloqueada por Control de
aplicaciones de Windows. No se ha deshabilitado esa protección. La compilación
PlatformIO local también quedó bloqueada al ejecutar el toolchain ARM instalado.
La CI de GitHub ejecuta sanitizers, pruebas, banco y compilación real para Nano.
Consultar los resultados de la CI del commit, no interpretar compilación con
stubs como validación de la placa. Web: 50 pruebas, TypeScript y build pasan.

Falta cargar y medir el robot físico, verificar las cinco posiciones de montaje
y repetir el banco amplio de 161 casos para aprobar rendimiento. Permanecen las
limitaciones físicas y el caso extremo de sensores bajos 1046 del traspaso base.

CI del primer commit experimental: [ejecución 39691380787](https://github.com/KevinVicenteR/Raton_laberinto/actions/runs/39691380787).
Los tres trabajos terminaron sin runner asignado ni pasos ejecutados; no hay
logs de compilación disponibles. Por tanto esa ejecución no valida ni rechaza
el código: queda pendiente ejecutar las pruebas nativas y de placa finales
en un entorno que permita iniciar los ejecutables y los trabajos de CI.
