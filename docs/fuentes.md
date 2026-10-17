# Conocimiento externo para el reentrenamiento

Cada dato lleva su fuente. «Verificado» = leído en el documento original; «secundario» = resumen de terceros.

## 1. Reglas de competencia (laberinto clásico, reglas IEE/IEEE y derivadas)
- 16×16 celdas de 180 mm; paredes de 50 mm de alto y 12 mm de grosor → pasillo ≈ 168 mm. Postes 12×12 mm.
- Meta: bloque central de 2×2 celdas (sin poste en medio). Salida en una esquina, con paredes en 3 lados.
- Todo poste toca al menos una pared (salvo el centro de la meta).
- Tolerancia de construcción: 5 % o 20 mm, lo que sea menor; huecos entre paredes ≤ 1 mm.
- Tiempos: «run» = salida→meta; volver a la salida no es run; «search time» desde la primera salida.
  En Japón hay un número limitado de runs, por lo que explorar con eficiencia es decisivo.
- Variantes: certámenes regionales usan 9×9 o pistas propias (p. ej., la pista del usuario: pasillo medido ≈ 192 mm).
- Fuentes: [Wikipedia: Micromouse](https://en.wikipedia.org/wiki/Micromouse) · [Reglas IEEE R2 SAC 2020](https://attend.ieee.org/r2sac-2020/wp-content/uploads/sites/175/2020/01/MicroMouse_Rules_2020.pdf) · [Reglas Robochallenge](https://robochallenge.ro/regulation/micromouse) · [Reglamento de Łódź](https://www.skaner.p.lodz.pl/sumochallenge/files/en/micromouse.pdf)

## 2. Hojas de datos de los componentes
### VL53L0X (verificado, ST DocID029104 Rev 1)
- Campo de visión (FOV) de 25°. Periodo mínimo de medida de 8 ms; presupuesto de tiempo mínimo de 20 ms.
- El firmware usa `setMeasurementTimingBudget(20000)` → perfil «High speed», con precisión de ±5 %.
- Precisión (desviación estándar) con 33 ms en interiores: 4 % (blanco a 120 cm), 7 % (gris a 70 cm).
  A 20 ms es peor. Se modela σ ≈ max(3 mm, 5 % de la distancia) con colas más pesadas sobre madera oscura.
- Tasa de detección garantizada: ≥ 94 %, es decir, hasta un 6 % de lecturas inválidas aun con blanco detectado.
- Calibración de offset recomendada a 10 cm; el offset deriva con la tensión, la temperatura y el cristal protector.
- Diafonía (cross-talk) = retorno del cristal protector; error proporcional a diafonía/señal, crece con la distancia.
- Haz ancho → un poste aislado puede parecer una pared lejana, y una esquina puede devolver un eco de la pared vecina.
- Fuente: [Hoja de datos VL53L0X](https://www.st.com/en/imaging-and-photonics-solutions/vl53l0x.html) (copia leída: elecrow VL53L0X.pdf)

### BMI270 (secundario: página de producto de Bosch)
- Offset de velocidad cero: ±0,5 °/s típico. Usuarios reportan hasta 3 °/s, por lo que hay que calibrarlo en reposo.
- Densidad de ruido: 0,007–0,008 °/s/√Hz.
- Fuente: [Bosch BMI270](https://bosch-sensortec.com/products/motion-sensors/imus/bmi270.html) · [foro Bosch, offset alto](https://community.bosch-sensortec.com/t5/MEMS-sensors-forum/BMI270-gyroscope-x-axis-zero-rate-offset-out-of-spec/m-p/8328)

### N20 1:100 con encoder magnético (secundario)
- 6 V, ~100 mA sin carga, ~200 mA bloqueado (Adafruit ADA4639). Encoder Hall de 2 canales.
- El firmware usa 7 × 4 × 100 = 2800 cuentas/vuelta y una rueda de 34 mm; se verifica midiendo un tramo conocido.
- Holgura de la reductora y asimetría entre motores (el usuario midió ≈ 7 %).
- Fuente: [N20 1:100 (The Pi Hut)](https://thepihut.com/products/adafruit-n20-dc-motor-with-magnetic-encoder-6v-with-1-100-gear-ratio)

### DRV8833
- Puente H doble. Por debajo de cierto PWM los N20 no arrancan (zona muerta). Ya está modelada como `TURN_PWM_MIN`.

## 3. Calibración (procedimientos recomendados)
1. **Ruedas y encoders:** mm/cuenta = π·D / cuentas por vuelta. Avanzar una distancia conocida y corregir D.
   Para diferencial existe el método UMBmark (cuadrado en ambos sentidos) para la asimetría.
2. **Giróscopo:** sesgo con el ratón quieto (ya se hace: `calibrateStill`). Escala comparando con giros
   conocidos o con el rumbo de los encoders.
3. **Sensores de pared:** calibrar a la distancia de uso. Con dos frontales se mide el ángulo contra la pared
   (lineal en ±20°). Aquí hay un solo frontal más laterales diagonales: el ángulo lateral se identifica con
   pasillo conocido (`K<mm>`, ≈31°).
4. **ToF:** offset a 10 cm con un blanco plano; diafonía si hay cubierta.
- Fuentes: [Micromouse Online: giróscopo](https://micromouseonline.com/2011/05/08/setting-up-a-gyro-on-the-micromouse/) · [Micromouse Online: ángulo con frontales](https://micromouseonline.com/2011/10/13/measure-angular-error-using-front-sensors/) · [calibración de sensores de pared](https://micromouseonline.com/2011/05/07/easier-wall-sensor-calibration/) · [error de alineación de sensores](https://micromouseonline.com/2011/05/29/wall-sensor-alignment-error/)

## 4. Estrategias y algoritmos de competidores
- **Flood fill (Bellman)** sobre las paredes conocidas, re-inundando con cada pared nueva. Es la base casi universal.
- **Coste de giro:** flood fill modificado que penaliza giros (artículo IPB). El equipo Zeetah asigna un tiempo
  por tipo de giro; la ruta más corta no siempre es la más rápida. Hay registros de ganadores que eligieron
  la ruta con menos giros.
- **Adachi (1984):** método 0 = ir y volver salida↔meta; método 1 = buscar las celdas sin explorar *sobre la
  mejor ruta calculada* (resolver dos veces por decisión).
- **Condición de parada:** cuando la mejor ruta salida→meta ya no pasa por ninguna celda sin explorar, la ruta
  óptima está demostrada y se pasa a la carrera rápida.
- **Vuelta a la salida por otro camino:** explorar en el regreso, con prudencia (no puntúa).
- **Poda de zonas muertas:** «dead-end exclusion» y «osmosis» (Beihang). Excluyen zonas que no pueden
  contener la ruta más corta.
- **Inundar hasta la salida**, no solo hasta el ratón, para poder decidir el fin temprano de la búsqueda.
- **Carrera rápida:** perfil trapezoidal de velocidad en rectas largas; diagonales solo con un control muy fino
  (aquí no: N20 lentos y laterales ToF).
- Fuentes: [Adachi (Micromouse Online)](https://micromouseonline.com/2008/05/27/adachi-micromouse-maze-solving-algorithms/) · [resolución más rápida](https://micromouseonline.com/micromouse-book/mazes-and-maze-solving/faster-maze-solving/) · [navegación](https://micromouseonline.com/micromouse-book/navigation/) · [solver diagonal Zeetah](https://micromouseonline.com/2013/01/02/diagonal-solver-introduction-and-flooding/) · [flood fill con coste de giro (IPB)](https://bibliotecadigital.ipb.pt/entities/publication/f7234f09-0056-41e0-81aa-8b9df00174cc) · [exploración mejorada (Beihang)](https://research.buaa.edu.cn/zh/publications/an-improved-algorithm-of-the-exploring-process-in-micromouse-comp/) · [flood fill (UCI)](https://ieee.ics.uci.edu/micromouse/floodfill.html) · [avances recientes](https://www.i-programmer.info/news/105-artificial-intelligence/16351-spectacular-advances-in-micromouse-performance-.html)

## 5. Vídeos de concursos
No puedo ver vídeos de YouTube directamente. Lo que se observa en ellos (exploración inicial lenta, carrera
rápida con rectas aceleradas, regreso por otro camino, varias carreras con la ruta ya demostrada) llega aquí a
través de las crónicas y análisis escritos citados arriba. Los dos vídeos del usuario (modos 3 y 4) ya se
usaron: roces, giros imprecisos, celda fija.
