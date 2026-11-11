#include "benchmarks/cli.hpp"
#include "benchmarks/planning_benchmarks.hpp"

int main(int argc, char** argv) {
  const auto options = mpb::planning::parseCommandLine(argc, argv);
  if (options.describe) {
    mpb::planning::describeConfig();
  } else {
    mpb::planning::runBenchmark(options);
  }
  return 0;
}
