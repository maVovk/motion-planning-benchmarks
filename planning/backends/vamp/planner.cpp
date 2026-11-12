#include "backends/vamp/planner.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <random>
#include <ratio>
#include <vamp/collision/environment.hh>
#include <vamp/collision/factory.hh>
#include <vamp/planning/rrtc.hh>
#include <vamp/planning/rrtc_settings.hh>
#include <vamp/random/xorshift.hh>
#include <vamp/robots/ur5.hh>
#include <vamp/vector.hh>
#include <vector>

#include "core/robot.hpp"
#include "core/types.hpp"

namespace mpb::planning {

namespace {

using Robot = vamp::robots::UR5;
using Environment = vamp::collision::Environment<vamp::FloatVector<kSimdWidth>>;
using Clock = std::chrono::steady_clock;

Robot::Configuration packed(const Configuration& configuration) {
  alignas(vamp::FloatVectorAlignment)
      std::array<float, Robot::Configuration::num_scalars_rounded>
          values{};
  std::copy(configuration.begin(), configuration.end(), values.begin());
  return Robot::Configuration(values.data());
}
}  // namespace

struct VampPlanner::Impl {
  Environment environment;

  static Environment makeEnvironment(const std::vector<Sphere>& obstacles) {
    vamp::collision::Environment<float> input;
    for (const auto& sphere : obstacles) {
      input.spheres.emplace_back(vamp::collision::factory::sphere::array(
          sphere.center, sphere.radius));
    }

    input.sort();

    return Environment(input);
  }

  explicit Impl(const std::vector<Sphere>& obstacles)
      : environment(makeEnvironment(obstacles)) {}
};

VampPlanner::VampPlanner(const std::vector<Sphere>& obstacles)
    : impl_(std::make_unique<Impl>(obstacles)) {}

VampPlanner::~VampPlanner() = default;

bool VampPlanner::valid(const Configuration& configuration) const {
  if (!withinBounds(configuration)) {
    return false;
  }

  Robot::ConfigurationBlock<kSimdWidth> block;
  for (size_t i = 0; i < kJoints; ++i) {
    block[i] = vamp::FloatVector<kSimdWidth>::fill(configuration[i]);
  }

  return Robot::fkcc(impl_->environment, block);
}

PlanResult VampPlanner::solve(const Problem& problem,
                              const PlanningConfig& config,
                              std::uint32_t seed) {
  static_assert(PlanningConfig::motion_resolution == Robot::resolution);

  const auto setup_start = Clock::now();

  std::mt19937_64 keys(seed);
  const auto key1 = keys();
  const auto key2 = keys();
  auto rng = std::make_shared<vamp::rng::XORShift<Robot>>(key1, key2);

  vamp::planning::RRTCSettings settings;
  settings.range = config.range;
  settings.max_iterations = config.max_iterations;
  settings.max_samples = config.max_samples;

  const auto start = packed(problem.start);
  const auto goal = packed(problem.goal);

  PlanResult result;
  result.setup_us =
      std::chrono::duration<double, std::micro>(Clock::now() - setup_start)
          .count();

  const auto solve_start = Clock::now();
  const auto plan =
      vamp::planning::RRTC<Robot, kSimdWidth, Robot::resolution>::solve(
          start, goal, impl_->environment, settings, rng);
  result.solve_us =
      std::chrono::duration<double, std::micro>(Clock::now() - solve_start)
          .count();

  result.iterations = plan.iterations;
  for (const auto& configuration : plan.path) {
    const auto values = configuration.to_array();
    Configuration output;
    std::copy_n(values.begin(), kJoints, output.begin());
    result.path.push_back(output);
  }

  return result;
}

}  // namespace mpb::planning
