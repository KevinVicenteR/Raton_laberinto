#pragma once
#include <stdint.h>
#include "Wire.h"
#define SSD1306_SWITCHCAPVCC 2
#define SSD1306_WHITE 1
struct Adafruit_SSD1306 {
  Adafruit_SSD1306(int, int, TwoWire*, int) {}
  bool begin(int, uint8_t) { return true; }
  void setTextSize(int) {}
  void setTextColor(int) {}
  void clearDisplay() {}
  void display() {}
  void setCursor(int, int) {}
  void print(const char*) {}
};
