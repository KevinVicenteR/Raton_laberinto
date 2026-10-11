// OLED SSD1306 128x64 en I2C. Si no esta conectada, todo sigue funcionando.
#pragma once

#include <Arduino.h>

class Display {
 public:
  bool begin();
  bool ok() const { return ok_; }
  // Hasta 8 lineas de 21 caracteres. Las lineas nullptr se dejan vacias.
  void show(const char* l0, const char* l1 = nullptr, const char* l2 = nullptr, const char* l3 = nullptr,
            const char* l4 = nullptr, const char* l5 = nullptr, const char* l6 = nullptr, const char* l7 = nullptr);

 private:
  bool ok_ = false;
};
