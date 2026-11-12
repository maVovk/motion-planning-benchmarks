#include "core/problem.hpp"

#include <cstddef>
#include <cstdint>
#include <random>
#include <stdexcept>

#include "backends/coal/checker.hpp"
#include "backends/vamp/planner.hpp"
#include "core/constants.hpp"
#include "core/geometry.hpp"
#include "core/robot.hpp"
#include "core/types.hpp"

namespace mpb::planning {

namespace {

Configuration sampleEndpoint(std::mt19937& random,
                             std::uniform_real_distribution<float>& joint,
                             CoalChecker& coal, VampPlanner& vamp,
                             const ProblemConfig& config) {
  for (std::uint32_t attempt = 0; attempt < config.max_generation_attempts;
       ++attempt) {
    Configuration q;
    for (auto& angle : q) {
      angle = joint(random);
    }

    const bool valid = coal.valid(q) && vamp.valid(q);

    if (valid) {
      return q;
    }
  }

  throw std::runtime_error("Cannot generate a valid endpoint");
}

}  // namespace

Problem generateProblem(uint32_t obstacle_count, uint32_t seed,
                        const ProblemConfig& config) {
  std::mt19937 random(seed);
  std::uniform_real_distribution<float> joint(-PI, PI);
  std::uniform_real_distribution<float> unit(0, 1);

  CoalChecker self_checker({});
  VampPlanner self_vamp({});

  Problem problem{
      seed,
      {},
      sampleEndpoint(random, joint, self_checker, self_vamp, config),
      {}};

  for (uint32_t attempts = 0; attempts <= config.max_generation_attempts;
       ++attempts) {
    if (attempts == config.max_generation_attempts) {
      throw std::runtime_error("Cannot separate endpoints");
    }

    problem.goal =
        sampleEndpoint(random, joint, self_checker, self_vamp, config);

    if (configurationDistance(problem.start, problem.goal) >=
        config.min_endpoint_distance) {
      break;
    }
  }

  const auto start_spheres = robotSpheres(problem.start);
  const auto goal_spheres = robotSpheres(problem.goal);

  for (uint32_t attempts = 0; attempts <= config.max_generation_attempts; ++attempts) {
    if (attempts == config.max_generation_attempts) {
      throw std::runtime_error("Cannot generate obstacle scene");
    }

    Sphere sphere;
    for (size_t i = 0; i < 3; ++i) {
      sphere.center[i] =
          config.lower[i] + unit(random) * (config.upper[i] - config.lower[i]); 
    }
    sphere.radius = config.min_radius +
                    unit(random) * (config.max_radius - config.min_radius);

    bool overlaps = false;
    for (const auto& robot : start_spheres) {
      overlaps |= spheresOverlap(sphere, robot, 0.005);
    }
    for (const auto& robot : goal_spheres) {
      overlaps |= spheresOverlap(sphere, robot, 0.005);
    }

    if (!overlaps) {
      problem.obstacles.push_back(sphere);
    }

    if (problem.obstacles.size() == obstacle_count) {
      break;
    }
  }

  return problem;
}

}  // namespace mpb::planning
