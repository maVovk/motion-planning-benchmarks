#include <benchmark/benchmark.h>

#include <string_view>

#include "benchmarks/cli.hpp"
#include "benchmarks/planning_benchmarks.hpp"

int main(int argc, char** argv) {
  if (argc == 1 || std::string_view(argv[1]).starts_with("--benchmark") ||
      std::string_view(argv[1]) == "--help") {
    benchmark::Initialize(&argc, argv);
    if (benchmark::ReportUnrecognizedArguments(argc, argv)) {
      return 1;
    }
    mpb::planning::registerBenchmarks();
    benchmark::RunSpecifiedBenchmarks();
    benchmark::Shutdown();
    return 0;
  }

  const auto options = mpb::planning::parseCommandLine(argc, argv);
  if (options.describe) {
    mpb::planning::describeConfig();
  } else {
    mpb::planning::runBenchmark(options);
  }
  return 0;
}
