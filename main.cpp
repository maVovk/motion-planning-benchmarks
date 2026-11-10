#include "coal_benchmarks.hpp"
#include "config.hpp"

#include <benchmark/benchmark.h>

#include <exception>
#include <iostream>

int main(int argc, char** argv) {
  try {
    benchmark::Initialize(&argc, argv);
    if (benchmark::ReportUnrecognizedArguments(argc, argv)) return 1;
    mpb::benchmarks::registerBenchmarks(mpb::benchmarks::BenchmarkConfig{});
    benchmark::RunSpecifiedBenchmarks();
    benchmark::Shutdown();
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
