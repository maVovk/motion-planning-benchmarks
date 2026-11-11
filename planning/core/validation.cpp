#include "core/validation.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <random>
#include <stdexcept>
#include <vector>

#include "backends/coal/checker.hpp"
#include "backends/vamp/planner.hpp"
#include "core/robot.hpp"
#include "core/types.hpp"

namespace mpb::planning {

namespace {

bool overlaps(const Sphere& a, const Sphere& b) {
  double squared = 0;
  for (size_t i = 0; i < 3; ++i) {
    double delta = static_cast<double>(a.center[i]) - b.center[i];
    squared += delta * delta;
  }

  double radius = static_cast<double>(a.radius) + b.radius;
  return squared <= radius * radius;
}
}  // namespace

// what does this function do?
bool analyticValid(const Configuration& configuration,
                   const std::vector<Sphere>& obstacles) {
  if (!withinBounds(configuration)) {
    return false;
  }

  const auto robot = robotSpheres(configuration);

  for (const auto& [a, b] : selfCollisionPairs()) {
    if (overlaps(robot[a], robot[b])) {
      return false;
    }
  }

  for (const auto& sphere : robot) {
    for (const auto& obstacle : obstacles) {
      if (overlaps(sphere, obstacle)) {
        return false;
      }
    }
  }

  return true;
}

void compareCheckers(const Problem& problem, std::uint32_t samples) {
  CoalChecker coal(problem.obstacles);
  VampPlanner vamp(problem.obstacles);

  std::mt19937 random(problem.seed ^ 0x94AF20u);
  std::uniform_real_distribution<float> joint(-3.14159265F, 3.14159265F);

  for (std::uint32_t i = 0; i < samples + 2; ++i) {
    Configuration q;
    if (i == 0) {
      q = problem.start;
    } else if (i == 1) {
      q = problem.goal;
    } else {
      for (auto& angle : q) {
        angle = joint(random);
      }
    }

    const bool expected = analyticValid(q, problem.obstacles);
    if (coal.valid(q) != expected || vamp.valid(q) != expected) {
      throw std::runtime_error("Collision checker disagreement");
    }
  }
}

ValidationResult validatePath(const Problem& problem, const Path& path,
                              double step) {
  ValidationResult result;
  if (path.size() < 2 ||
      configurationDistance(path.front(), problem.start) > 1e-4 ||
      configurationDistance(path.back(), problem.goal) > 1e-4) {
    return result;
  }

  CoalChecker checker(problem.obstacles);
  for (size_t segment = 0; segment + 1 < path.size(); ++segment) {
    const auto length = configurationDistance(path[segment], path[segment + 1]);
    result.path_length += length;

    const auto samples = std::max(1u, std::uint32_t(std::ceil(length / step)));
    for (std::uint32_t i = 0; i <= samples; ++i) {
      const auto q =
          interpolate(path[segment], path[segment + 1], double(i) / samples);
      ++result.checked_states;
      if (!checker.valid(q) || !analyticValid(q, problem.obstacles)) {
        return result;
      }
    }
  }

  result.valid = true;
  return result;
}

}  // namespace mpb::planning
