# Calibración completa, PID estable y reinicio

Estas funciones corresponden a `experimental/autonomia-v3`. No cambian `arquitectura_v2`.

## Uso en la web

Los tres modos de navegación se conservan. **CALIBRACIÓN COMPLETA** es una rutina de preparación independiente, disponible cuando la placa anuncia `CAPS RESET_TOTAL CALIB_COMPLETA`. También se solicita con `calall` por serie/BLE. La rutina no inicia después una misión: queda en la salida para seleccionar uno de los tres modos.

Antes de pulsarla: colocar el robot **centrado**, alineado en la salida, sobre el suelo de uso, con dos paredes laterales paralelas del ancho configurado (`cellMm-wallMm`), al menos una celda libre delante y una pared frontal visible a menos de 600 mm. Medir una vez dimensiones del cuerpo, posiciones de sensores y ancho del pasillo: no son observables todas por sí solas. Retirar las manos. El giro requiere que el círculo barrido por el cuerpo quepa con 6 mm de margen.

1. Calibrar sesgo del giroscopio en reposo y comprobar encoders y disponibilidad de sensores.
2. Exigir lecturas estables de los cinco VL53L0X; barrer lentamente ambos lados y corregir offsets del frontal y diagonales. Separar los registros de ambos lados evita truncar el barrido lento.
3. Ajustar offsets laterales con la colocación centrada y el ancho conocido; activar ambos laterales.
4. Avanzar aproximadamente 0.65 celdas en dos etapas lentas, de 50 y 90 mm/s. Contrastar avance de encoders con la reducción de distancia frontal. Desacuerdos mayores del 15 % impiden guardar.
5. Volver por ese corto tramo y contrastar los cinco sensores (diferencia <=8 mm), odometría (<=3 mm) y rumbo (<=2°). No volver automáticamente tras PARAR o fallo de IMU/encoders.
6. Solo tras verificar el regreso y dos velocidades estables, estimar kS/kV y la escala efectiva de distancia de ruedas, guardar en flash y conservar el mapa anterior. CPR, batalla, dimensiones físicas y constante temporal del motor no se inventan. El diámetro guardado es una escala efectiva, no una medición física independiente.

`CAL_COMPLETA_OK` confirma regreso verificado y escritura en flash. La web lee después los parámetros nuevos. `CAL_COMPLETA_NO_APLICADA` restaura parámetros/mapa anteriores; la posición física debe revisarse porque un fallo puede impedir volver. Cada etapa tiene tiempo limitado y la orden PARAR sigue disponible. No se promete regresar ante cualquier avería.

La rutina identifica feedforward y verifica el seguimiento; **no identifica automáticamente los seis coeficientes PID ni realiza la calibración óptica completa de fábrica ST**. El sesgo IMU se recalibra en cada arranque; no se presenta como un ajuste permanente de sensibilidad.

## Perfil PID prudente

Con `precisionMode=1`, cada arranque aplica estos límites superiores; respeta valores inferiores ya configurados:

| Parámetro | Límite |
| --- | --- |
| kpFwd | 0.009 duty/mm |
| kpRot | 0.015 duty/grado |
| kiFwd, kiRot | 0.006 en sus unidades respectivas |
| kSteer | 0.12 grados/mm |
| steerMaxDeg | 2.5 grados |
| accel | 900 mm/s² |
| rotSpeed | 240 grados/s |
| rotAccel | 1800 grados/s² |

La placa anuncia `PID_ESTABLE` al aplicarlo. Son valores iniciales conservadores, no ganancias óptimas identificadas en el robot. Los parámetros pueden inspeccionarse con `params`/sync. `precisionMode=0` permite experimentar con las ganancias configuradas, manteniendo las protecciones por sensores.

La derivada angular usa un filtro de 25 ms en rectas y 12 ms en giros. Se añade filtro de 15 ms a la derivada de avance para no convertir cuantización del encoder en pulsos de torque. Se conserva antiwindup por integración condicional. Cuando las consignas están cumplidas, los errores son menores de 0.8 mm/0.4° y la velocidad es baja, se anula la salida y se vacía el integral; pequeñas variaciones del giroscopio no deben reactivar la compensación de rozamiento. El filtro lateral mantiene límite de variación y banda muerta.

La protección frontal adicional utiliza distancia medida, dimensiones del cuerpo, velocidad, latencia y frenado: no exige que la pared coincida con la celda que dice el mapa. Dos muestras cercanas activan recuperación sin añadir por sí solas una pared al mapa. Los laterales activan la protección al quedar menos de 6 mm entre el cuerpo y una pared. Se activa su control cuando ambos responden en la autoprueba. Esta protección se aplica en avance recto; maniobras de recuperación, giros y diagonales conservan sus propias verificaciones. No sustituye una geometría bien medida ni garantiza cero contactos.

## Reiniciar todo

**REINICIAR TODO** cancela órdenes pendientes, manda PARAR y solicita `resetall`. La placa detiene los motores inmediatamente y difiere el borrado hasta salir de la pila de navegación. Borra parámetros y mapa persistentes, restaura valores de fábrica y reinicia el MCU, eliminando pose, filtros, aprendizaje RAM y registro offline. No arranca solo.

El ACK solo confirma recepción. La web borra parámetros, mapas 2D/3D, curvas, historial, trayectoria y selección anterior únicamente al recibir `REINICIO_TOTAL_OK`; ignora tramas antiguas hasta el nuevo anuncio de firmware. Un error de flash se informa sin anunciar éxito; los dos borrados no constituyen una transacción atómica. Si se pierde la confirmación durante una desconexión, volver a sincronizar y comprobar el estado; se puede reintentar el reinicio. La primera vinculación Bluetooth autorizada se conserva en el navegador.

## Verificación

El simulador incluye regresiones de vibración del mando, reposo con ruido IMU, pasillo con ruido/asimetría, obstáculo frontal independiente del mapa, calibración con ida/regreso y rechazo de sensores ausentes. Las pruebas web comprueban que no se borre la sesión solo por recibir ACK ni se dibujen tramas antiguas después del borrado.

La compilación de comprobación de firmware usa stubs de PC: no sustituye una compilación ARM ni una prueba con la placa. El compilador ARM de PlatformIO sigue sin poder ejecutarse en este Windows (`El sistema no puede ejecutar el programa especificado`). No se ha cargado esta versión al Nano.

Resultado de esta revisión: CTest núcleo/autonomía pasa; web 59 pruebas, build y lint pasan. En pruebas sintéticas, RMS de cambios del mando común 0.123597 → 0.011100 (~91 %) y diferencial 0.063875 → 0.006266 (~90 %). Esto mide ruido de mando simulado, no vibración real. El pasillo con sensores ruidosos y motores asimétricos llega a meta sin contactos; la calibración nominal verifica regreso sin contactos. No certifica todos los laberintos, batería/suelo ni el PID del robot físico.
