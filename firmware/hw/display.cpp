#include "hw/display.h"

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>

#include "config.h"

namespace {
Adafruit_SSD1306 oled(128, 64, &Wire, -1);
}

bool Display::begin() {
  ok_ = oled.begin(SSD1306_SWITCHCAPVCC, pins::ADDR_OLED);
  if (ok_) {
    oled.setTextSize(1);
    oled.setTextColor(SSD1306_WHITE);
    oled.clearDisplay();
    oled.display();
  }
  return ok_;
}

void Display::show(const char* l0, const char* l1, const char* l2, const char* l3, const char* l4, const char* l5) {
  if (!ok_) return;
  const char* lines[6] = {l0, l1, l2, l3, l4, l5};
  oled.clearDisplay();
  for (int i = 0; i < 6; ++i) {
    if (!lines[i]) continue;
    oled.setCursor(0, i * 10 + (i ? 4 : 0));
    oled.print(lines[i]);
  }
  oled.display();
}
