#include "coal_benchmarks.hpp"

#include "config.hpp"
#include "mpb/collision.hpp"
#include "mpb/manager.hpp"
#include "mpb/memory.hpp"
#include "mpb/scene.hpp"
#include "workloads.hpp"

#include <benchmark/benchmark.h>

#include <cstddef>
#include <string>

namespace mpb::benchmarks {
namespace {
// Memory probes run outside timed loops. These are retained process-heap deltas,
// including scene vectors, rather than peak allocations or per-query allocations.
void recordMemory(benchmark::State& state, const std::string& name,
                  std::size_t count, const BenchmarkConfig& config) {
  const auto baseline = memorySnapshot();
  Scene scene(count, config.scene);
  const auto with_scene = memorySnapshot();
  auto manager = buildManager(name, scene.pointers, config.hash);
  const auto with_index = memorySnapshot();
  state.counters["scene_heap_bytes"] = with_scene.heap_bytes - baseline.heap_bytes;
  state.counters["index_heap_bytes"] = with_index.heap_bytes - with_scene.heap_bytes;
  state.counters["scene_rss_bytes"] = with_scene.rss_bytes - baseline.rss_bytes;
  state.counters["index_rss_bytes"] = with_index.rss_bytes - with_scene.rss_bytes;
}

void benchmarkScene(benchmark::State& state, const BenchmarkConfig& config) {
  for (auto _ : state) {
    Scene scene(state.range(0), config.scene);
    benchmark::DoNotOptimize(scene.pointers.data());
    benchmark::ClobberMemory();
  }
}

void benchmarkInitialization(benchmark::State& state, const std::string& name,
                             const BenchmarkConfig& config) {
  recordMemory(state, name, state.range(0), config);
  Scene scene(state.range(0), config.scene);
  for (auto _ : state) {
    auto manager = buildManager(name, scene.pointers, config.hash);
    benchmark::DoNotOptimize(manager.get());
    state.PauseTiming();
    manager.reset();
    state.ResumeTiming();
  }
}

void benchmarkCollision(benchmark::State& state, const std::string& name,
                        Workload workload, const BenchmarkConfig& config) {
  Scene scene(state.range(0), config.scene);
  auto manager = buildManager(name, scene.pointers, config.hash);
  auto queries = makeQueries(scene, workload, config.scene, config.query);
  validateQueries(*manager, scene, queries, config.scene);
  CollisionCallback callback;
  std::size_t candidates = 0, hits = 0;
  for (const auto& query : queries) {
    callback.init();
    manager->collide(query.get(), &callback);
    candidates += callback.candidates;
    hits += callback.hits;
  }
  state.counters["candidates_per_query"] = double(candidates) / queries.size();
  state.counters["hit_fraction"] = double(hits) / queries.size();
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
  auto sizes = [&config](benchmark::internal::Benchmark* benchmark) {
    for (int count : config.sphere_counts) benchmark->Arg(count);
    return benchmark->Unit(benchmark::kMicrosecond);
  };
  sizes(benchmark::RegisterBenchmark("scene/create_destroy", [config](benchmark::State& state) {
    benchmarkScene(state, config);
  }));
  for (const std::string name : kManagers) {
    sizes(benchmark::RegisterBenchmark(("initialize/" + name).c_str(),
        [name, config](benchmark::State& state) {
          benchmarkInitialization(state, name, config);
        }));
    for (std::size_t i = 0; i < kWorkloadNames.size(); ++i) {
      const auto workload = static_cast<Workload>(i);
      sizes(benchmark::RegisterBenchmark(("collision/" + name + "/" + kWorkloadNames[i]).c_str(),
          [name, workload, config](benchmark::State& state) {
            benchmarkCollision(state, name, workload, config);
          }));
    }
  }
}

}  // namespace mpb::benchmarks
