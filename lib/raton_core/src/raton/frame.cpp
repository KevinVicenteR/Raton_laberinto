#include "raton/frame.h"

#include <stdio.h>
#include <string.h>

namespace raton {

uint8_t crc8(const char* data, size_t len) {
  uint8_t crc = 0;
  for (size_t i = 0; i < len; ++i) {
    crc ^= static_cast<uint8_t>(data[i]);
    for (int b = 0; b < 8; ++b) crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0x07) : static_cast<uint8_t>(crc << 1);
  }
  return crc;
}

size_t frameLine(uint16_t seq, const char* payload, char* out, size_t outSize) {
  const int head = snprintf(out, outSize, "$%u,%s", static_cast<unsigned>(seq), payload);
  if (head < 0 || static_cast<size_t>(head) + 3 >= outSize) return 0;
  const uint8_t crc = crc8(out + 1, static_cast<size_t>(head) - 1);
  snprintf(out + head, outSize - head, "*%02X", crc);
  return static_cast<size_t>(head) + 3;
}

FrameResult parseFrame(const char* line, uint16_t& seq, char* out, size_t outSize) {
  if (line[0] != '$') return FrameResult::Plain;
  const char* star = strrchr(line, '*');
  const char* comma = strchr(line, ',');
  if (!star || !comma || comma > star || strlen(star) != 3) return FrameResult::Malformed;
  unsigned crc = 0;
  if (sscanf(star + 1, "%2X", &crc) != 1) return FrameResult::Malformed;
  if (crc8(line + 1, static_cast<size_t>(star - line - 1)) != crc) return FrameResult::BadCrc;
  unsigned s = 0;
  for (const char* p = line + 1; p < comma; ++p) {
    if (*p < '0' || *p > '9') return FrameResult::Malformed;
    s = s * 10 + static_cast<unsigned>(*p - '0');
    if (s > 65535) return FrameResult::Malformed;
  }
  const size_t n = static_cast<size_t>(star - comma - 1);
  if (n + 1 > outSize) return FrameResult::Malformed;
  memcpy(out, comma + 1, n);
  out[n] = 0;
  seq = static_cast<uint16_t>(s);
  return FrameResult::Ok;
}

}  // namespace raton
