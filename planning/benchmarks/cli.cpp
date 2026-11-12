#include "benchmarks/cli.hpp"

#include <stdexcept>
#include <string_view>

namespace mpb::planning {

CommandLineOptions parseCommandLine(int argc, char** argv) {
  CommandLineOptions options;
  if (argc == 2 && std::string_view(argv[1]) == "--describe") {
    options.describe = true;

    return options;
  }

  if (argc > 1 && std::string_view(argv[1]) == "--correctness") {
    options.correctness = true;
    --argc;
    ++argv;
  }

  if (argc < 5) {
    throw std::invalid_argument(
        "usage: planning_benchmark [--benchmark_* options] or "
        "[--correctness] {vamp|ompl} N scene_seed planner_seed "
        "[--export FILE]");
  }

  options.planner = argv[1];
  const auto& planner = options.planner;
  options.obstacle_count = std::stoul(argv[2]);
  options.scene_seed = std::stoul(argv[3]);
  options.planner_seed = std::stoul(argv[4]);

  if (options.obstacle_count == 0) {
    throw std::invalid_argument("N must be positive");
  }
  if (planner != "vamp" && planner != "ompl") {
    throw std::invalid_argument("Unknown planner");
  }

  for (int i = 5; i < argc; i += 2) {
    if (i + 1 >= argc) {
      throw std::invalid_argument("Missing option value");
    }

    if (std::string(argv[i]) == "--export") {
      options.export_file = argv[i + 1];
    } else {
      throw std::invalid_argument("Unknown option");
    }
  }

  return options;
}

}  // namespace mpb::planning
