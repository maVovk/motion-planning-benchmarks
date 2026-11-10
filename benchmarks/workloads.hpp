#pragma once

#include "config.hpp"
#include "mpb/manager.hpp"
#include "mpb/scene.hpp"

#include <array>
#include <memory>
#include <vector>

namespace mpb::benchmarks {
enum class Workload { OutsideMiss, Hit, Random };
constexpr std::array<const char*, 3> kWorkloadNames{"outside_miss", "hit", "random"};


std::vector<std::unique_ptr<Object>> makeQueries(const Scene& scene,
    Workload workload, const SceneConfig& config, const QueryConfig& query_config);

// Sphere-specific oracle: checks complete counts and early-exit collision flags.
void validateQueries(Manager& manager, const Scene& scene,
    const std::vector<std::unique_ptr<Object>>& queries, const SceneConfig& config);
}  // namespace mpb::benchmarks
