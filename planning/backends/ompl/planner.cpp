#include "backends/ompl/planner.hpp"

#include <ompl/base/MotionValidator.h>
#include <ompl/base/PlannerStatus.h>
#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/util/Console.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <ratio>
#include <utility>

#include "backends/coal/checker.hpp"
#include "core/robot.hpp"
#include "core/types.hpp"
#include "runtime/deadline.hpp"

namespace mpb::planning {

namespace {

namespace ob = ompl::base;
using Clock = std::chrono::steady_clock;

Configuration configuration(const ob::State* state) {
  Configuration q;
  const auto* values = state->as<ob::RealVectorStateSpace::StateType>()->values;
  std::copy_n(values, kJoints, q.begin());

  return q;
}

class SeededSampler : public ob::RealVectorStateSampler {
 public:
  SeededSampler(const ob::StateSpace* space, std::uint32_t seed)
      : ob::RealVectorStateSampler(space) {
    rng_.setLocalSeed(seed);
  }
};

class SeededRRTConnect : public ompl::geometric::RRTConnect {
 public:
  SeededRRTConnect(const ob::SpaceInformationPtr& si, std::uint32_t seed)
      : RRTConnect(si) {
    rng_.setLocalSeed(seed);
  }
};

// Same nominal density as VAMP, including its interleaved/raked sample
// locations. Coal evaluates them one at a time, while VAMP processes eight
// lanes together.
class CoalMotionValidator : public ob::MotionValidator {
 public:
  CoalMotionValidator(const ob::SpaceInformationPtr& si, CoalChecker& checker,
                      std::uint32_t resolution)
      : MotionValidator(si), checker_(checker), resolution_(resolution) {}

  bool checkMotion(const ob::State* first,
                   const ob::State* second) const override {
    std::pair<ob::State*, double> unused{nullptr, 0};
    return checkMotion(first, second, unused);
  }

  bool checkMotion(const ob::State* first, const ob::State* second,
                   std::pair<ob::State*, double>& last_valid) const override {
    const auto start = configuration(first);
    const auto goal = configuration(second);
    const auto batches = std::max(
        1u, std::uint32_t(std::ceil(configurationDistance(start, goal) *
                                    resolution_ / kSimdWidth)));

    // Scan in increasing interpolation order so last_valid satisfies OMPL's
    // contract.
    const auto samples = batches * kSimdWidth;
    for (std::uint32_t i = 1; i <= samples; ++i) {
      const auto q = interpolate(start, goal, double(i) / samples);
      if (!checker_.valid(q)) {
        ++invalid_;
        last_valid.second = double(i - 1) / samples;
        if (last_valid.first) {
          si_->getStateSpace()->interpolate(first, second, last_valid.second,
                                            last_valid.first);
        }
        return false;
      }
    }

    ++valid_;
    return true;
  }

 private:
  CoalChecker& checker_;
  std::uint32_t resolution_;
};
}  // namespace

struct OmplPlanner::Impl {
  std::shared_ptr<CoalChecker> checker;

  explicit Impl(const std::vector<Sphere>& obstacles)
      : checker(std::make_shared<CoalChecker>(obstacles)) {}
};

OmplPlanner::OmplPlanner(const std::vector<Sphere>& obstacles)
    : impl_(std::make_unique<Impl>(obstacles)) {}

OmplPlanner::~OmplPlanner() = default;

bool OmplPlanner::valid(const Configuration& configuration) const {
  return impl_->checker->valid(configuration);
}

PlanResult OmplPlanner::solve(const Problem& problem,
                              const PlanningConfig& config,
                              std::uint32_t seed) {
  const auto setup_start = Clock::now();

  ompl::msg::setLogLevel(ompl::msg::LOG_ERROR);
  const auto checker = impl_->checker;

  auto space = std::make_shared<ob::RealVectorStateSpace>(kJoints);
  ob::RealVectorBounds bounds(kJoints);

  // Exact float bounds used by the VAMP UR5 sampler; no periodic wrap in either
  // backend.
  bounds.setLow(-3.1415927410125732);
  bounds.setHigh(3.1415927410125732);
  space->setBounds(bounds);

  space->setStateSamplerAllocator([seed](const ob::StateSpace* state_space) {
    return std::make_shared<SeededSampler>(state_space, seed);
  });

  auto si = std::make_shared<ob::SpaceInformation>(space);
  si->setStateValidityChecker([checker](const ob::State* state) {
    return checker->valid(configuration(state));
  });
  si->setMotionValidator(std::make_shared<CoalMotionValidator>(
      si, *checker, config.motion_resolution));
  si->setup();

  ob::ScopedState<> start(space), goal(space);
  for (size_t i = 0; i < kJoints; ++i) {
    start[i] = problem.start[i];
    goal[i] = problem.goal[i];
  }

  auto definition = std::make_shared<ob::ProblemDefinition>(si);
  definition->setStartAndGoalStates(start, goal);

  auto planner = std::make_shared<SeededRRTConnect>(si, seed);
  planner->setRange(config.range);
  planner->setProblemDefinition(definition);
  planner->setup();

  PlanResult result;
  result.setup_us =
      std::chrono::duration<double, std::micro>(Clock::now() - setup_start)
          .count();

  SolveDeadline deadline(config.timeout_seconds);
  const auto solve_start = Clock::now();
  const auto status = planner->solve(
      ob::timedPlannerTerminationCondition(config.timeout_seconds));
  result.solve_us =
      std::chrono::duration<double, std::micro>(Clock::now() - solve_start)
          .count();

  if (status == ob::PlannerStatus::EXACT_SOLUTION) {
    const auto path = std::dynamic_pointer_cast<ompl::geometric::PathGeometric>(
        definition->getSolutionPath());
    for (const auto* state : path->getStates()) {
      result.path.push_back(configuration(state));
    }
  }

  return result;
}

}  // namespace mpb::planning
