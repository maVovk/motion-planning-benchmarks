#include <benchmark/benchmark.h>

#include <cstddef>
#include <exception>
#include <iostream>
#include <string_view>

#include "coal_benchmarks.hpp"
#include "config.hpp"
#include "memory_measurements.hpp"
#include "mpb/manager.hpp"

void runMemoryBench(mpb::benchmarks::BenchmarkConfig& config) {
  const auto measurements = mpb::benchmarks::measureMemory(config);
  std::cout << "manager,sphere_count,scene_heap_bytes,index_heap_bytes,"
               "scene_rss_bytes,index_rss_bytes\n";
  for (const auto& measurement : measurements) {
    std::cout << mpb::kManagerNames[static_cast<size_t>(measurement.manager)]
              << ',' << measurement.sphere_count << ','
              << measurement.scene.heap_bytes << ','
              << measurement.index.heap_bytes << ','
              << measurement.scene.rss_bytes << ','
              << measurement.index.rss_bytes << '\n';
  }
}

int main(int argc, char** argv) {
  mpb::benchmarks::BenchmarkConfig config{};
  mpb::benchmarks::validateConfig(config);

  if (argc > 1 && std::string_view(argv[1]) == "--memory") {
    if (argc != 2) {
      std::cerr << "--memory запускается отдельно без Google Benchmark\n";
      return 1;
    }

    runMemoryBench(config);
    return 0;
  }

  benchmark::Initialize(&argc, argv);

  if (benchmark::ReportUnrecognizedArguments(argc, argv)) {
    return 1;
  }

  mpb::benchmarks::registerBenchmarks(config);
  benchmark::RunSpecifiedBenchmarks();
  benchmark::Shutdown();
}