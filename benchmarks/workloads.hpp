#pragma once

#include <array>
#include <cstddef>
#include <memory>
#include <string_view>
#include <vector>

#include "config.hpp"
#include "mpb/manager.hpp"
#include "mpb/scene.hpp"

namespace mpb::benchmarks {
enum class Workload { OutsideMiss, Hit, Random };
constexpr std::array<std::string_view, 3> kWorkloadNames{"outside_miss", "hit",
                                                         "random"};

std::vector<std::unique_ptr<coal::CollisionObject>> makeQueries(
    const Scene& scene, Workload workload, const SceneConfig& config,
    const QueryConfig& query_config);

struct QueryStatistics {
  size_t candidates = 0;
  size_t hits = 0;
};

QueryStatistics validateAndMeasureQueries(Manager& manager, const Scene& scene,
                     const std::vector<std::unique_ptr<coal::CollisionObject>>& queries,
                     const SceneConfig& config);
}  // namespace mpb::benchmarks
