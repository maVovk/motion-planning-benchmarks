#include "coal_benchmarks.hpp"

#include <benchmark/benchmark.h>
#include <fmt/format.h>

#include <cstddef>
#include <string_view>

#include "config.hpp"
#include "mpb/collision.hpp"
#include "mpb/manager.hpp"
#include "mpb/scene.hpp"
#include "workloads.hpp"

namespace mpb::benchmarks {
namespace {
void benchmarkScene(benchmark::State& state, const BenchmarkConfig& config) {
  for (auto _ : state) {
    Scene scene(state.range(0), config.scene);
    benchmark::DoNotOptimize(scene.objects.data());
    benchmark::ClobberMemory();
  }
}

void benchmarkManagerInitialization(benchmark::State& state, ManagerType type,
                                    const BenchmarkConfig& config) {
  Scene scene(state.range(0), config.scene);
  const auto pointers = scene.objectPointers();

  for (auto _ : state) {
    auto manager = buildManager(type, pointers, config.hash);
    benchmark::DoNotOptimize(manager.get());
  }
}

void benchmarkCollision(benchmark::State& state, ManagerType type,
                        Workload workload, const BenchmarkConfig& config) {
  Scene scene(state.range(0), config.scene);
  auto manager = buildManager(type, scene.objectPointers(), config.hash);
  auto queries = makeQueries(scene, workload, config.scene, config.query);
  const auto statistics = validateAndMeasureQueries(*manager, scene, queries, config.scene);

  state.counters["candidates_per_query"] = static_cast<double>(statistics.candidates) / queries.size();
  state.counters["hit_fraction"] = static_cast<double>(statistics.hits) / queries.size();

  CollisionCallback callback;
  for (auto _ : state) {
    for (const auto& query : queries) {
      callback.init();
      manager->collide(query.get(), &callback);
      benchmark::DoNotOptimize(callback.hits);
    }
  }
  state.SetItemsProcessed(state.iterations() * queries.size());
}

}  // namespace

void registerBenchmarks(const BenchmarkConfig& config) {
  benchmark::RegisterBenchmark(
      "scene/initialize",
      [config](benchmark::State& state) { benchmarkScene(state, config); })
      ->RangeMultiplier(10)
      ->Range(1, 10000)
      ->Unit(benchmark::kMicrosecond);

  for (size_t index = 0; index < kManagerNames.size(); ++index) {
    const auto type = static_cast<ManagerType>(index);
    const std::string_view name = kManagerNames[index];

    benchmark::RegisterBenchmark(fmt::format("initialize/{}", name),
                                 [type, config](benchmark::State& state) {
                                   benchmarkManagerInitialization(state, type,
                                                                  config);
                                 })
        ->RangeMultiplier(10)
        ->Range(1, 10000)
        ->Unit(benchmark::kMicrosecond);

    for (size_t i = 0; i < kWorkloadNames.size(); ++i) {
      const auto workload = static_cast<Workload>(i);
      benchmark::RegisterBenchmark(
          fmt::format("collision/{}/{}", name, kWorkloadNames[i]),
          [type, workload, config](benchmark::State& state) {
            benchmarkCollision(state, type, workload, config);
          })
          ->RangeMultiplier(10)
          ->Range(1, 10000)
          ->Unit(benchmark::kMicrosecond);
    }
  }
}

}  // namespace mpb::benchmarks
