#pragma once

#include <cstdint>

namespace mpb {
struct MemorySnapshot {
  std::int64_t heap_bytes;
  std::int64_t rss_bytes;
};

MemorySnapshot memorySnapshot();

}  // namespace mpb
