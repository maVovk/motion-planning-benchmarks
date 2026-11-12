#pragma once

#include "benchmarks/cli.hpp"

namespace mpb::planning {

void describeConfig();
void registerBenchmarks();
void runBenchmark(const CommandLineOptions& options);

}  // namespace mpb::planning
