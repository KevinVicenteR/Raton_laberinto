#include "mapa/map_storage.h"

#include <mbed.h>
#include <FlashIAPBlockDevice.h>
#include <TDBStore.h>
#include <cmath>
#include <cstring>

namespace {
// Dos bancos de 16 KiB: TDBStore conserva el registro previo durante escritura/GC.
FlashIAPBlockDevice block(0xF8000, 0x8000);
mbed::TDBStore store(&block);
bool ready() {
  static bool initialized = false;
  if (!initialized) initialized = store.init() == 0;
  return initialized;
}
constexpr size_t kHeader = 20;
uint8_t buffer[kHeader + raton::CellMap::kSerializedMax];
uint32_t checksum(const uint8_t* data, size_t size) {
  uint32_t crc = 0xffffffffu;
  for (size_t i = 0; i < size; ++i) {
    crc ^= data[i];
    for (int b = 0; b < 8; ++b) crc = (crc >> 1) ^ (0xedb88320u & (0u - (crc & 1u)));
  }
  return ~crc;
}
}
namespace mapStorage {
bool save(const raton::CellMap& map, const raton::TrackGeometry& geometry) {
  if (!ready()) return false;
  const size_t size = map.serialize(buffer + kHeader, sizeof(buffer) - kHeader);
  if (!size) return false;
  const float values[] = {geometry.corridorMm(), geometry.cellMm(), geometry.sideSin()};
  buffer[0] = 'T'; buffer[1] = 'R'; buffer[2] = 'K'; buffer[3] = 1;
  memcpy(buffer + 4, values, sizeof(values));
  const uint32_t deadzone = geometry.deadzoneMm();
  memcpy(buffer + 16, &deadzone, 4);
  const uint32_t crc = checksum(buffer, kHeader + size);
  // El CRC externo incluye geometria y mapa; se anexa al registro atomico.
  uint8_t record[sizeof(buffer) + 4];
  memcpy(record, buffer, kHeader + size);
  memcpy(record + kHeader + size, &crc, 4);
  return store.set("track", record, kHeader + size + 4, 0) == 0;
}
bool load(raton::CellMap& map, raton::TrackGeometry& geometry) {
  if (!ready()) return false;
  uint8_t record[sizeof(buffer) + 4];
  size_t size = 0;
  if (store.get("track", record, sizeof(record), &size) != 0 || size < kHeader + 16) return false;
  uint32_t crc;
  memcpy(&crc, record + size - 4, 4);
  if (record[0] != 'T' || record[1] != 'R' || record[2] != 'K' || record[3] != 1 ||
      crc != checksum(record, size - 4)) return false;
  float values[3];
  uint32_t deadzone;
  memcpy(values, record + 4, sizeof(values));
  memcpy(&deadzone, record + 16, 4);
  if (!std::isfinite(values[0]) || !std::isfinite(values[1]) || !std::isfinite(values[2]) ||
      values[0] < 128 || values[0] > 388 || fabsf(values[1] - values[0] - geometry.params().wallMm) > 0.1f ||
      values[2] < 0.3f || values[2] > 0.98f || deadzone > 150) return false;
  if (!map.deserialize(record + kHeader, size - kHeader - 4)) return false;
  geometry.setCorridor(values[0]);
  geometry.setSideSin(values[2]);
  geometry.setDeadzone(deadzone);
  geometry.setManual(true);
  return true;
}
bool erase() {
  if (!ready()) return false;
  mbed::KVStore::info_t info;
  if (store.get_info("track", &info) == MBED_ERROR_ITEM_NOT_FOUND) return true;
  return store.remove("track") == 0;
}
}
