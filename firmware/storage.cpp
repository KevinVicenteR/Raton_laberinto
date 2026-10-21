#include "storage.h"

// El orden importa: mbed.h trae Arduino.h y la configuracion de la placa (MBED_CONF_*) que necesita
// FlashIAPBlockDevice.h; incluirlo despues rompe la compilacion con el nucleo mbed de Arduino.
#include <mbed.h>
// Valores por defecto de mbed (por si una version del nucleo no los define): base y tamano los da el
// constructor de abajo.
#ifndef MBED_CONF_FLASHIAP_BLOCK_DEVICE_BASE_ADDRESS
#define MBED_CONF_FLASHIAP_BLOCK_DEVICE_BASE_ADDRESS 0xFFFFFFFF
#endif
#ifndef MBED_CONF_FLASHIAP_BLOCK_DEVICE_SIZE
#define MBED_CONF_FLASHIAP_BLOCK_DEVICE_SIZE 0
#endif
#include <FlashIAPBlockDevice.h>
#include <TDBStore.h>
#include <string.h>
#include <stddef.h>

namespace {

FlashIAPBlockDevice block(0xF8000, 0x8000);
mbed::TDBStore store(&block);

bool ready() {
  static bool initialized = false;
  if (!initialized) initialized = store.init() == 0;
  return initialized;
}

uint32_t crc32(const uint8_t* d, size_t n) {
  uint32_t crc = 0xffffffffu;
  for (size_t i = 0; i < n; ++i) {
    crc ^= d[i];
    for (int b = 0; b < 8; ++b) crc = (crc >> 1) ^ (0xedb88320u & (0u - (crc & 1u)));
  }
  return ~crc;
}

struct Header {
  uint32_t magic;
  uint32_t size;
  uint32_t crc;
};

const uint32_t kParamsMagic = 0x52503031;  // "RP01"
const uint32_t kMapMagic = 0x524D3031;     // "RM01"

uint8_t buffer[sizeof(Header) + sizeof(raton::Maze) + 16];

bool save(const char* key, uint32_t magic, const void* data, size_t size) {
  if (!ready() || size + sizeof(Header) > sizeof(buffer)) return false;
  Header h{magic, static_cast<uint32_t>(size), crc32(static_cast<const uint8_t*>(data), size)};
  memcpy(buffer, &h, sizeof(h));
  memcpy(buffer + sizeof(h), data, size);
  return store.set(key, buffer, sizeof(h) + size, 0) == 0;
}

bool load(const char* key, uint32_t magic, void* data, size_t size) {
  if (!ready()) return false;
  size_t got = 0;
  if (store.get(key, buffer, sizeof(buffer), &got) != 0 || got != sizeof(Header) + size) return false;
  Header h;
  memcpy(&h, buffer, sizeof(h));
  if (h.magic != magic || h.size != size || h.crc != crc32(buffer + sizeof(h), size)) return false;
  memcpy(data, buffer + sizeof(h), size);
  return true;
}

bool erase(const char* key) {
  if (!ready()) return false;
  mbed::KVStore::info_t info;
  if (store.get_info(key, &info) == MBED_ERROR_ITEM_NOT_FOUND) return true;
  return store.remove(key) == 0;
}

}  // namespace

namespace storage {

bool saveParams(const raton::Params& p) { return save("params", kParamsMagic, &p, sizeof(p)); }
bool loadParams(raton::Params& p) {
  raton::Params tmp;
  if (!load("params", kParamsMagic, &tmp, sizeof(tmp)) &&
      !load("params", kParamsMagic, &tmp, offsetof(raton::Params, autoGeometryConfirmed)) &&
      !load("params", kParamsMagic, &tmp, offsetof(raton::Params, sideEnabled)) &&
      !load("params", kParamsMagic, &tmp, offsetof(raton::Params, precisionMode)) &&
      !load("params", kParamsMagic, &tmp, offsetof(raton::Params, motorBalance))) return false;
  p = tmp;
  return true;
}
bool saveMap(const raton::Maze& m) { return save("map", kMapMagic, &m, sizeof(m)); }
bool loadMap(raton::Maze& m) {
  static raton::Maze tmp;
  if (!load("map", kMapMagic, &tmp, sizeof(tmp))) return false;
  m = tmp;
  return true;
}
bool eraseMap() { return erase("map"); }
bool eraseParams() { return erase("params"); }

}  // namespace storage

