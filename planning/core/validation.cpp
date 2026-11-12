#include "core/validation.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "backends/coal/checker.hpp"
#include "core/constants.hpp"
#include "core/geometry.hpp"
#include "core/robot.hpp"
#include "core/types.hpp"

namespace mpb::planning {

bool analyticValid(const Configuration& configuration,
                   const std::vector<Sphere>& obstacles) {
  if (!withinBounds(configuration)) {
    return false;
  }

  const auto robot = robotSpheres(configuration);

  for (const auto& [a, b] : selfCollisionPairs()) {
    if (spheresOverlap(robot[a], robot[b])) {
      return false;
    }
  }

  for (const auto& sphere : robot) {
    for (const auto& obstacle : obstacles) {
      if (spheresOverlap(sphere, obstacle)) {
        return false;
      }
    }
  }

  return true;
}

ValidationResult validatePath(const Problem& problem, const Path& path,
                              double step, std::optional<Path>& sampled_path,
                              ValidationMode mode) {
  ValidationResult result;
  if (sampled_path) {
    sampled_path->clear();
  }
  if (path.size() < 2 ||
      configurationDistance(path.front(), problem.start) > kEndpointTolerance ||
      configurationDistance(path.back(), problem.goal) > kEndpointTolerance) {
    return result;
  }

  CoalChecker checker(problem.obstacles);
  for (size_t segment = 0; segment + 1 < path.size(); ++segment) {
    const auto length = configurationDistance(path[segment], path[segment + 1]);
    result.path_length += length;

    const auto samples = std::max(1u, static_cast<uint32_t>(std::ceil(length / step)));
    for (uint32_t i = 0; i <= samples; ++i) {
      const auto q =
          interpolate(path[segment], path[segment + 1], double(i) / samples);
      ++result.checked_states;
      if (!checker.valid(q) ||
          (mode == ValidationMode::Correctness &&
           !analyticValid(q, problem.obstacles))) {
        return result;
      }
      if (sampled_path.has_value() && (segment == 0 || i > 0)) {
        sampled_path->push_back(q);
      }
    }
  }

  result.valid = true;
  return result;
}

}  // namespace mpb::planning
