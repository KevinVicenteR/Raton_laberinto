# Fuentes consultadas

- Convocatoria Concurso de Robótica SECIHTI Yucatán 2026, categoría b) Robot ratón.
- Pololu, foros sobre el VL53L0X: presupuesto de 20 ms en modo "high speed", modo continuo "back-to-back"
  (startContinuous(0)) con menos sobrecarga, y límite de señal de 0.25 MCPS por defecto.
  https://forum.pololu.com/t/vl53l0x-timing-budget-settings/16934 ,
  https://forum.pololu.com/t/high-speed-with-vl53l0x/16585
- Arduino, librería Arduino_BMI270_BMM150: el giroscopio va fijo a 99.84 Hz y ±2000 dps.
  https://docs.arduino.cc/libraries/arduino_bmi270_bmm150
- Texas Instruments, hoja de datos del DRV8833: tabla de control PWM (slow decay = una entrada en alto y la
  otra con PWM) y foro E2E sobre el mayor par a baja velocidad en slow decay.
  https://www.ti.com/document-viewer/DRV8833/datasheet/detailed_description ,
  https://e2e.ti.com/support/applications/motor_drivers/f/38/t/251780
- Micromouse Online: inundación, búsqueda y ruta con coste de giros.
  https://micromouseonline.com/2013/01/06/diagonal-solver-flooding/
