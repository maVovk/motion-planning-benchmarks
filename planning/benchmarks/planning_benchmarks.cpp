#include "benchmarks/planning_benchmarks.hpp"

#include <chrono>
#include <iostream>
#include <nlohmann/json.hpp>
#include <stdexcept>

#include "backends/ompl/planner.hpp"
#include "backends/vamp/planner.hpp"
#include "benchmarks/trajectory.hpp"
#include "core/config.hpp"
#include "core/problem.hpp"
#include "core/robot.hpp"
#include "core/validation.hpp"

namespace mpb::planning {

namespace {

using Json = nlohmann::json;
using Clock = std::chrono::steady_clock;

template <typename Planner>
PlanResult solveWithPlanner(const Problem& problem,
                            const PlanningConfig& config, std::uint32_t seed) {
  const auto setup_start = Clock::now();
  Planner planner(problem.obstacles);
  const double environment_us =
      std::chrono::duration<double, std::micro>(Clock::now() - setup_start)
          .count();

  auto result = planner.solve(problem, config, seed);
  result.setup_us += environment_us;

  return result;
}

PlanResult solveProblem(const Problem& problem, const PlanningConfig& config,
                        const std::string& planner,
                        std::uint32_t planner_seed) {
  if (planner == "ompl") {
    return solveWithPlanner<OmplPlanner>(problem, config, planner_seed);
  }

  return solveWithPlanner<VampPlanner>(problem, config, planner_seed);
}

void validateAndReportScene(const Problem& problem,
                            const PlanningConfig& config) {
  const auto count = problem.obstacles.size();
  const auto scene_seed = problem.seed;

  // Fail on disagreement between coal, VAMP, and the analytic sphere oracle.
  // Check both endpoints as well as deterministic random configurations.
  constexpr std::uint32_t random_samples = 2000;
  compareCheckers(problem, random_samples);

  Json scene{{"obstacles", count},
             {"scene_seed", scene_seed},
             {"checked_states", random_samples + 2},
             {"valid", true},
             {"start", problem.start},
             {"goal", problem.goal},
             {"spheres", Json::array()},
             {"direct_path_valid",
              validatePath(problem, Path{problem.start, problem.goal},
                           config.validation_step)
                  .valid}};

  for (const auto& sphere : problem.obstacles) {
    scene["spheres"].push_back(
        {{"center", sphere.center}, {"radius", sphere.radius}});
  }

  std::cout << scene << std::endl;
}

// Repetition supplies enough work for external sampling of fast solves.
void repeatSolves(const Problem& problem, const PlanningConfig& config,
                  const CommandLineOptions& options) {
  const auto end =
      Clock::now() + std::chrono::duration<double>(options.profile_seconds);
  size_t solves = 0;

  while (Clock::now() < end) {
    auto repeated =
        solveProblem(problem, config, options.planner, options.planner_seed);

    if (repeated.path.empty()) {
      throw std::runtime_error("Profile solve failed");
    }

    ++solves;
  }

  std::cout << Json{{"planner", options.planner},
                    {"obstacles", problem.obstacles.size()},
                    {"profile_solves", solves}}
            << std::endl;
}
}  // namespace

void describeConfig() {
  const PlanningConfig planning;
  const ProblemConfig problem;

  std::cout << Json{{"range", planning.range},
                    {"motion_resolution", planning.motion_resolution},
                    {"validation_step", planning.validation_step},
                    {"timeout_seconds", planning.timeout_seconds},
                    {"max_iterations", planning.max_iterations},
                    {"max_samples", planning.max_samples},
                    {"obstacle_lower", problem.lower},
                    {"obstacle_upper", problem.upper},
                    {"radius_min", problem.min_radius},
                    {"radius_max", problem.max_radius},
                    {"robot_spheres", kRobotSpheres},
                    {"self_collision_pairs", selfCollisionPairs().size()}}
            << std::endl;
}

void runBenchmark(const CommandLineOptions& options) {
  const auto& planner = options.planner;
  const auto count = options.obstacle_count;
  const auto scene_seed = options.scene_seed;
  const auto planner_seed = options.planner_seed;
  const auto& export_file = options.export_file;

  const PlanningConfig config;

  const auto problem = generateProblem(count, scene_seed);
  if (planner == "check") {
    validateAndReportScene(problem, config);
    return;
  }

  const auto result = solveProblem(problem, config, planner, planner_seed);
  const auto validation =
      validatePath(problem, result.path, config.validation_step);

  if (!result.path.empty() && !validation.valid) {
    throw std::runtime_error("Returned path failed dense validation");
  }

  if (!export_file.empty() && validation.valid) {
    exportTrajectory(export_file, problem, result, planner);
  }

  if (options.profile_seconds > 0) {
    repeatSolves(problem, config, options);
    return;
  }

  std::cout << Json{{"planner", planner},
                    {"obstacles", count},
                    {"scene_seed", scene_seed},
                    {"planner_seed", planner_seed},
                    {"success", validation.valid},
                    {"setup_us", result.setup_us},
                    {"solve_us", result.solve_us},
                    {"path_length", validation.path_length},
                    {"path_states", result.path.size()},
                    {"validation_states", validation.checked_states},
                    {"iterations", result.iterations ? Json(*result.iterations)
                                                     : Json(nullptr)}}
            << std::endl;
}

}  // namespace mpb::planning
