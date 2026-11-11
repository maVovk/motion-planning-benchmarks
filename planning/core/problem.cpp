#include "core/problem.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <random>
#include <stdexcept>

#include "backends/coal/checker.hpp"
#include "backends/vamp/planner.hpp"
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

    const bool valid = coal.valid(q);
    if (valid != vamp.valid(q)) {
      throw std::runtime_error("Endpoint self-collision disagreement");
    }

    if (valid) {
      return q;
    }
  }

  throw std::runtime_error("Cannot generate a valid endpoint");
}

bool intersectsEndpoint(const Sphere& obstacle, const Sphere& robot) {
  double squared = 0;
  for (size_t i = 0; i < 3; ++i) {
    squared += std::pow(double(obstacle.center[i]) - robot.center[i], 2);
  }

  return squared <= std::pow(double(obstacle.radius) + robot.radius + 0.005, 2);
}
}  // namespace

Problem generateProblem(std::uint32_t obstacle_count, std::uint32_t seed,
                        const ProblemConfig& config) {
  std::mt19937 random(seed);
  std::uniform_real_distribution<float> joint(-3.14159265F, 3.14159265F);
  std::uniform_real_distribution<float> unit(0, 1);

  CoalChecker self_checker({});
  VampPlanner self_vamp({});

  Problem problem{
      seed,
      {},
      sampleEndpoint(random, joint, self_checker, self_vamp, config),
      sampleEndpoint(random, joint, self_checker, self_vamp, config)};
  std::uint32_t attempts = 0;
  while (configurationDistance(problem.start, problem.goal) <
         config.min_endpoint_distance) {
    if (++attempts == config.max_generation_attempts) {
      throw std::runtime_error("Cannot separate endpoints");
    }
    problem.goal =
        sampleEndpoint(random, joint, self_checker, self_vamp, config);
  }

  // Condition the random obstacles only on valid endpoints, never on planner
  // success.
  const auto start_spheres = robotSpheres(problem.start);
  const auto goal_spheres = robotSpheres(problem.goal);
  for (attempts = 0; problem.obstacles.size() < obstacle_count; ++attempts) {
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
      overlaps |= intersectsEndpoint(sphere, robot);
    }
    for (const auto& robot : goal_spheres) {
      overlaps |= intersectsEndpoint(sphere, robot);
    }

    if (!overlaps) {
      problem.obstacles.push_back(sphere);
    }
  }

  return problem;
}

}  // namespace mpb::planning
