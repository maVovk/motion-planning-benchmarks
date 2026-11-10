#pragma once

#include "mpb/manager.hpp"
#include "mpb/scene.hpp"

#include <array>
#include <cstddef>

namespace mpb::benchmarks {
struct QueryConfig {
  double outside_x = 30.0;
  std::size_t queries_per_workload = 128;
};

struct BenchmarkConfig {
  SceneConfig scene;
  HashConfig hash;
  QueryConfig query;
  std::array<int, 5> sphere_counts{1, 10, 100, 1000, 10000};
};
}  // namespace mpb::benchmarks
