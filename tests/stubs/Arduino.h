// Cabeceras minimas para COMPILAR el firmware en el PC (comprobacion de tipos y sintaxis, no se ejecuta).
#pragma once
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
enum { INPUT, OUTPUT, INPUT_PULLUP, LOW = 0, HIGH = 1, CHANGE = 2 };
enum { A0 = 14, A1, A2, A3, A4, A5, LED_BUILTIN = 13 };
enum PinName { P0_0 = 0 };
inline void pinMode(int, int) {}
inline void digitalWrite(int, int) {}
inline int digitalRead(int) { return 0; }
inline uint32_t millis() { return 0; }
inline uint32_t micros() { return 0; }
inline void delay(uint32_t) {}
inline void noInterrupts() {}
inline void interrupts() {}
inline int digitalPinToInterrupt(int p) { return p; }
inline PinName digitalPinToPinName(int) { return P0_0; }
inline void attachInterrupt(int, void (*)(), int) {}
inline void NVIC_SystemReset() {}
struct SerialStub {
  void begin(long) {}
  int available() { return 0; }
  int read() { return -1; }
  void println(const char*) {}
  void print(const char*) {}
  explicit operator bool() const { return true; }
};
extern SerialStub Serial;
