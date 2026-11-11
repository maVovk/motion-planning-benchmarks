#include "memory_measurements.hpp"

#include "config.hpp"
#include "mpb/manager.hpp"
#include "mpb/memory.hpp"
#include "mpb/scene.hpp"

#include <array>
#include <cstddef>
#include <vector>

namespace mpb::benchmarks {
namespace {
MemoryMeasurement measureCase(ManagerType type, size_t count,
                              const BenchmarkConfig& config) {
  const auto baseline = memorySnapshot();
  Scene scene(count, config.scene);
  const auto pointers = scene.objectPointers();
  const auto with_scene = memorySnapshot();
  auto manager = buildManager(type, pointers, config.hash);
  const auto with_index = memorySnapshot();
  return {type, count,
          {with_scene.heap_bytes - baseline.heap_bytes,
           with_scene.rss_bytes - baseline.rss_bytes},
          {with_index.heap_bytes - with_scene.heap_bytes,
           with_index.rss_bytes - with_scene.rss_bytes}};
}
}  // namespace

std::vector<MemoryMeasurement> measureMemory(const BenchmarkConfig& config) {
  constexpr std::array<size_t, 5> counts{1, 10, 100, 1000, 10000};
  std::vector<MemoryMeasurement> measurements;
  measurements.reserve(kManagerNames.size() * counts.size());
  // Initialize the snapshot machinery before taking per-case baselines.
  memorySnapshot();
  for (size_t index = 0; index < kManagerNames.size(); ++index) {
    for (const auto count : counts) {
      measurements.push_back(measureCase(static_cast<ManagerType>(index), count, config));
    }
  }
  return measurements;
}
}  // namespace mpb::benchmarks
