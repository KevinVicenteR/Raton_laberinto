#pragma once
#include "mbed.h"
namespace mbed {
struct TDBStore : KVStore {
  explicit TDBStore(FlashIAPBlockDevice*) {}
  int init() { return 0; }
  int set(const char*, const void*, size_t, uint32_t) { return 0; }
  int get(const char*, void*, size_t, size_t* got = nullptr, size_t = 0) { if (got) *got = 0; return -1; }
  int get_info(const char*, info_t*) { return 0; }
  int remove(const char*) { return 0; }
};
}
