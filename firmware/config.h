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
// Asignacion solicitada. Validar conflictos antes de accionar estos pines.
const int XSHUT_SIDE_LEFT = A0;
const int XSHUT_SIDE_RIGHT = A4;
const uint8_t ADDR_SIDE_LEFT = 0x33;
const uint8_t ADDR_SIDE_RIGHT = 0x34;

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

// Impedir que un XSHUT adicional reutilice el bus o una funcion existente.
constexpr bool occupied(int pin) {
  return pin == A4 || pin == A5 || pin == XSHUT_LEFT || pin == XSHUT_FRONT || pin == XSHUT_RIGHT ||
         pin == AIN1 || pin == AIN2 || pin == BIN1 || pin == BIN2 || pin == ENC_LEFT_A ||
         pin == ENC_LEFT_B || pin == ENC_RIGHT_A || pin == ENC_RIGHT_B || pin == LED_BUILTIN;
}
constexpr bool sidePinsConflict() {
  return (XSHUT_SIDE_LEFT >= 0 && occupied(XSHUT_SIDE_LEFT)) ||
         (XSHUT_SIDE_RIGHT >= 0 && occupied(XSHUT_SIDE_RIGHT)) ||
         (XSHUT_SIDE_LEFT >= 0 && XSHUT_SIDE_LEFT == XSHUT_SIDE_RIGHT);
}

}  // namespace pins

namespace wiring {

// Medido en el raton con la prueba de ruedas (orden "ruedas"): sentido de cada motor y encoder.
const bool INVERT_MOTOR_LEFT = true;
const bool INVERT_MOTOR_RIGHT = false;
const bool INVERT_ENC_LEFT = true;
const bool INVERT_ENC_RIGHT = false;
const bool ENCODERS_SWAPPED = true;  // el encoder del motor izquierdo esta en los pines del derecho
const float GYRO_SIGN = 1.0f;        // +1 si girar a la izquierda da rumbo positivo

// PWM del DRV8833 en "slow decay" (una entrada fija en alto y la otra con PWM invertido): la velocidad es
// casi proporcional al duty y el N20 tiene par a baja velocidad. 20 kHz no se oye.
const uint32_t PWM_PERIOD_US = 50;

const char* const BLE_NAME = "Raton";

}  // namespace wiring
