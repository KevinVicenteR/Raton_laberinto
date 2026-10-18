// Cola circular de lineas de texto para la telemetria (el control no espera nunca al Bluetooth). Si no
// hay sitio se tira la linea mas vieja. Sin dependencias: se prueba en el PC.
#pragma once

namespace raton {

class LineQueue {
 public:
  LineQueue(char* buffer, int size) : buf_(buffer), size_(size) {}

  int size() const { return size_; }
  int used() const { return (head_ - tail_ + size_) % size_; }
  bool empty() const { return head_ == tail_; }
  void clear() { head_ = tail_ = 0; }

  // Mete una linea (sin el salto). Devuelve cuantas lineas viejas tuvo que tirar, o -1 si no cabe.
  int push(const char* line, int len) {
    const int n = len + 1;  // con el salto de linea
    if (n >= size_) return -1;
    int dropped = 0;
    while (used() + n >= size_) {
      dropLine();
      ++dropped;
    }
    for (int i = 0; i < len; ++i) put(line[i]);
    put('\n');
    return dropped;
  }

  // Saca una linea entera (sin el salto) en out. Devuelve su longitud, o -1 si no hay.
  int pop(char* out, int max) {
    if (empty()) return -1;
    int n = 0;
    while (!empty()) {
      const char c = buf_[tail_];
      tail_ = (tail_ + 1) % size_;
      if (c == '\n') break;
      if (n < max - 1) out[n++] = c;
    }
    out[n] = 0;
    return n;
  }

 private:
  void put(char c) {
    buf_[head_] = c;
    head_ = (head_ + 1) % size_;
  }
  void dropLine() {
    while (!empty() && buf_[tail_] != '\n') tail_ = (tail_ + 1) % size_;
    if (!empty()) tail_ = (tail_ + 1) % size_;
  }

  char* buf_;
  int size_;
  int head_ = 0;
  int tail_ = 0;
};

}  // namespace raton
