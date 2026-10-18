void testFrame() {
  CHECK(raton::crc8("123456789", 9) == 0xF4);  // valor de referencia de CRC-8 (poly 0x07)
  char line[64], payload[32];
  const size_t n = raton::frameLine(7, "m3", line, sizeof(line));
  CHECK(n == strlen(line));
  uint16_t seq = 0;
  CHECK(raton::parseFrame(line, seq, payload, sizeof(payload)) == raton::FrameResult::Ok);
  CHECK(seq == 7 && std::string(payload) == "m3");
  line[3] = '4';  // "$7,m4*.." con el CRC de m3
  CHECK(raton::parseFrame(line, seq, payload, sizeof(payload)) == raton::FrameResult::BadCrc);
  CHECK(raton::parseFrame("m3", seq, payload, sizeof(payload)) == raton::FrameResult::Plain);
  CHECK(raton::parseFrame("$7,m3", seq, payload, sizeof(payload)) == raton::FrameResult::Malformed);
  CHECK(raton::parseFrame("$x,m3*00", seq, payload, sizeof(payload)) != raton::FrameResult::Ok);
  char small[6];
  CHECK(raton::frameLine(1, "una linea larga", small, sizeof(small)) == 0);
  // Misma trama que la web (frame(65535,'x')).
  raton::frameLine(65535, "x", line, sizeof(line));
  CHECK(raton::parseFrame(line, seq, payload, sizeof(payload)) == raton::FrameResult::Ok && seq == 65535);
}

void testLineQueue() {
  char buf[16];
  raton::LineQueue q(buf, sizeof(buf));
  char out[16];
  CHECK(q.pop(out, sizeof(out)) == -1);
  CHECK(q.push("abc", 3) == 0);
  CHECK(q.push("defg", 4) == 0);
  CHECK(q.pop(out, sizeof(out)) == 3 && std::string(out) == "abc");
  // Sin sitio: tira la linea mas vieja.
  CHECK(q.push("123456789A", 10) == 1);  // "defg" + esta no caben en 16
  CHECK(q.pop(out, sizeof(out)) == 10 && std::string(out) == "123456789A");
  CHECK(q.empty());
  CHECK(q.push("0123456789abcdef", 16) == -1);  // no cabe nunca
}

