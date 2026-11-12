#include "benchmarks/planning_benchmarks.hpp"

#include <benchmark/benchmark.h>
#include <fmt/format.h>
#include <nlohmann/json.hpp>

#include <chrono>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string_view>

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
                            const PlanningConfig& config, uint32_t seed) {
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
                        std::string_view planner, uint32_t planner_seed) {
  if (planner == "ompl") {
    return solveWithPlanner<OmplPlanner>(problem, config, planner_seed);
  } else if (planner == "vamp") {
    return solveWithPlanner<VampPlanner>(problem, config, planner_seed);
  }

  throw std::runtime_error("Unknown planner passed to the solve function");
}

void benchmarkPlanning(benchmark::State& state, std::string_view planner) {
  const PlanningConfig config;
  const auto problem = generateProblem(state.range(0), 42);
  double setup_us = 0;
  double path_length = 0;

  for (auto _ : state) {
    const auto result = solveProblem(problem, config, planner, 101);
    state.SetIterationTime(result.solve_us / 1e6);
    std::optional<Path> sampled_path = std::nullopt;
    const auto validation =
        validatePath(problem, result.path, config.validation_step, sampled_path);
    if (!validation.valid) {
      state.SkipWithError(result.path.empty()
                              ? "Planner did not find a path"
                              : "Returned path failed dense validation");
      break;
    }
    setup_us += result.setup_us;
    path_length += validation.path_length;
  }

  state.counters["setup_us"] =
      benchmark::Counter(setup_us, benchmark::Counter::kAvgIterations);
  state.counters["path_length"] =
      benchmark::Counter(path_length, benchmark::Counter::kAvgIterations);
}

}  // namespace

void registerBenchmarks() {
  for (const std::string_view planner : {"vamp", "ompl"}) {
    benchmark::RegisterBenchmark(
        fmt::format("Planning/{}", planner),
        [planner](benchmark::State& state) { benchmarkPlanning(state, planner); })
        ->RangeMultiplier(10)
        ->Range(1, 100)
        ->UseManualTime()
        ->Unit(benchmark::kMicrosecond);
  }
}

void describeConfig() {
  const PlanningConfig planning;
  const ProblemConfig problem;

  std::cout << Json{{"range", planning.range},
                    {"motion_resolution", planning.motion_resolution},
                    {"validation_step", planning.validation_step},
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
  const auto result = solveProblem(problem, config, planner, planner_seed);
  std::optional<Path> sampled_path =
      export_file.empty() ? std::nullopt : std::make_optional(Path{});
  const auto validation =
      validatePath(problem, result.path, config.validation_step, sampled_path,
                   options.correctness ? ValidationMode::Correctness
                                       : ValidationMode::Standard);

  if (!result.path.empty() && !validation.valid) {
    throw std::runtime_error("Returned path failed dense validation");
  }

  if (!export_file.empty() && validation.valid) {
    exportTrajectory(export_file, problem, result, planner, planner_seed,
                     sampled_path.value());
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
