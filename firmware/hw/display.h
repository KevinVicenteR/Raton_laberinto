// OLED SSD1306 128x64 (opcional). Solo se usa parado: refrescarla tarda ~25 ms y comparte el bus I2C con
// los laseres.
#pragma once

class Display {
 public:
  bool begin();
  bool ok() const { return ok_; }
  void show(const char* l0, const char* l1 = nullptr, const char* l2 = nullptr, const char* l3 = nullptr,
            const char* l4 = nullptr, const char* l5 = nullptr);

 private:
  bool ok_ = false;
};
