#pragma once

#include <cstdint>

#include "core/config.hpp"
#include "core/types.hpp"

namespace mpb::planning {

Problem generateProblem(std::uint32_t obstacle_count, std::uint32_t seed,
                        const ProblemConfig& config = {});

}  // namespace mpb::planning
