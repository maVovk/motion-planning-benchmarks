#pragma once

#include "config.hpp"
#include "mpb/manager.hpp"
#include "mpb/memory.hpp"

#include <cstddef>
#include <vector>

namespace mpb::benchmarks {
  struct MemoryMeasurement {
    ManagerType manager;
    size_t sphere_count;
    MemorySnapshot scene;
    MemorySnapshot index;
  };

  std::vector<MemoryMeasurement> measureMemory(const BenchmarkConfig& config);
}  // namespace mpb::benchmarks
