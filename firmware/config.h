// Pines y cableado del raton (Arduino Nano 33 BLE Sense Rev2). Los parametros de comportamiento estan en
// lib/raton_core/src/raton/params.h y se ajustan por Bluetooth/Serie sin recompilar ("set nombre valor").
#pragma once

#include <Arduino.h>

namespace pins {

// Laseres VL53L0X (I2C en A4/A5, compartido con la OLED). XSHUT para darles direccion uno a uno.
const uint8_t XSHUT_LEFT = A0;
const uint8_t XSHUT_FRONT = A1;
const uint8_t XSHUT_RIGHT = A2;
const uint8_t ADDR_LEFT = 0x30;
const uint8_t ADDR_FRONT = 0x31;
const uint8_t ADDR_RIGHT = 0x32;
const uint8_t ADDR_OLED = 0x3C;

// DRV8833: A = motor izquierdo, B = motor derecho.
const uint8_t AIN1 = 5;
const uint8_t AIN2 = 6;
const uint8_t BIN1 = 9;
const uint8_t BIN2 = 10;

// Encoders N20 (amarillo = A, verde = B). Alimentarlos a 3.3 V, NO a 5 V.
const uint8_t ENC_LEFT_A = 2;
const uint8_t ENC_LEFT_B = 3;
const uint8_t ENC_RIGHT_A = 4;
const uint8_t ENC_RIGHT_B = 7;

}  // namespace pins

namespace wiring {

// Medido en el raton con la prueba de ruedas (orden "ruedas"): sentido de cada motor y encoder.
const bool INVERT_MOTOR_LEFT = true;
const bool INVERT_MOTOR_RIGHT = false;
const bool INVERT_ENC_LEFT = true;
const bool INVERT_ENC_RIGHT = false;
const bool ENCODERS_SWAPPED = true;  // el encoder del motor izquierdo esta en los pines del derecho
const float GYRO_SIGN = 1.0f;        // +1 si girar a la izquierda da rumbo positivo

// Configuracion del firmware anterior (49e0e8a): los N20 del robot arrancaban
// con PWM de 500 Hz. La reescritura cambio a 20 kHz y frenado entre pulsos;
// se restaura la senal anterior ante la falta de arranque del motor izquierdo.
const uint32_t PWM_PERIOD_US = 2000;
const float PWM_MAX = 0.70f;  // motores de 6 V con bateria 2S

const char* const BLE_NAME = "Raton";

}  // namespace wiring
