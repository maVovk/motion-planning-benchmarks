#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "core/types.hpp"

namespace mpb::planning {

struct ValidationResult {
  bool valid = false;
  size_t checked_states = 0;
  double path_length = 0;
};
// Independent narrow-phase oracle using the same FK geometry and double
// arithmetic.
bool analyticValid(const Configuration& configuration,
                   const std::vector<Sphere>& obstacles);
void compareCheckers(const Problem& problem, std::uint32_t samples);
ValidationResult validatePath(const Problem& problem, const Path& path,
                              double step);

}  // namespace mpb::planning
