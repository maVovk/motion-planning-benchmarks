#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include "core/config.hpp"
#include "core/types.hpp"

namespace mpb::planning {

class OmplPlanner {
 public:
  explicit OmplPlanner(const std::vector<Sphere>& obstacles);

  ~OmplPlanner();

  bool valid(const Configuration& configuration) const;

  PlanResult solve(const Problem& problem, const PlanningConfig& config,
                   std::uint32_t seed);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace mpb::planning
