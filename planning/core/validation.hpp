#pragma once

#include <cstddef>
#include <optional>
#include <vector>

#include "core/types.hpp"

namespace mpb::planning {

enum class ValidationMode { Standard, Correctness };

struct ValidationResult {
  bool valid = false;
  size_t checked_states = 0;
  double path_length = 0;
};

bool analyticValid(const Configuration& configuration,
                   const std::vector<Sphere>& obstacles);
ValidationResult validatePath(const Problem& problem, const Path& path,
                              double step, std::optional<Path>& sampled_path,
                              ValidationMode mode = ValidationMode::Standard);

}  // namespace mpb::planning
