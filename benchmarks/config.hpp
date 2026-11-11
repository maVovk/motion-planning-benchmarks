#pragma once

#include "mpb/manager.hpp"
#include "mpb/scene.hpp"

#include <cstddef>

namespace mpb::benchmarks {
  struct QueryConfig {
    double outside_x = 30.0;
    size_t queries_per_workload = 128;
  };

  struct BenchmarkConfig {
    SceneConfig scene;
    HashConfig hash;
    QueryConfig query;
  };

  void validateConfig(const BenchmarkConfig& config);
}  // namespace mpb::benchmarks
