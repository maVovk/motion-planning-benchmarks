#pragma once

#include <string>

#include "core/types.hpp"

namespace mpb::planning {

void exportTrajectory(const std::string& filename, const Problem& problem,
                      const PlanResult& result, const std::string& planner);

}  // namespace mpb::planning
