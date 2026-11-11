#include "workloads.hpp"

#include <coal/data_types.h>

#include <cstddef>
#include <memory>
#include <random>
#include <stdexcept>
#include <vector>

#include "config.hpp"
#include "mpb/collision.hpp"
#include "mpb/manager.hpp"
#include "mpb/scene.hpp"

namespace mpb::benchmarks {
std::vector<std::unique_ptr<coal::CollisionObject>> makeQueries(
    const Scene& scene, Workload workload, const SceneConfig& config,
    const QueryConfig& query_config) {
  std::vector<std::unique_ptr<coal::CollisionObject>> queries;
  queries.reserve(query_config.queries_per_workload);

  std::seed_seq query_seed{config.seed, 1u};
  std::mt19937 random(query_seed);
  std::uniform_real_distribution<double> coordinate(
      -config.half_extent,
      config.half_extent);

  for (size_t i = 0; i < query_config.queries_per_workload; ++i) {
    const double x = coordinate(random);
    const double y = coordinate(random);
    const double z = coordinate(random);
    coal::Vec3s center(x, y, z);

    if (workload == Workload::OutsideMiss) {
      center.x() = query_config.outside_x;
    }
    if (workload == Workload::Hit) {
      center = scene.objects[i % scene.objects.size()]->getTranslation();
    }

    queries.push_back(makeSphere(center, config.sphere_radius));
  }
  return queries;
}

QueryStatistics validateAndMeasureQueries(Manager& manager, const Scene& scene,
                     const std::vector<std::unique_ptr<coal::CollisionObject>>& queries,
                     const SceneConfig& config) {
  const double diameter = 2 * config.sphere_radius;
  CollisionCallback callback;
  QueryStatistics statistics;

  for (const auto& query : queries) {
    size_t actual = 0;
    for (const auto& object : scene.objects) {
      actual += (object->getTranslation() - query->getTranslation()).squaredNorm() <=
                  diameter * diameter;
    }

    callback.stop_at_first = false;
    callback.init();
    manager.collide(query.get(), &callback);

    if (callback.hits != actual) {
      throw std::runtime_error("All-hits search mismatch");
    }

    callback.stop_at_first = true;
    callback.init();
    manager.collide(query.get(), &callback);

    if ((callback.hits > 0) != (actual > 0)) {
      throw std::runtime_error("Any-hit search mismatch");
    }
    statistics.candidates += callback.candidates;
    statistics.hits += callback.hits;
  }
  return statistics;
}

}  // namespace mpb::benchmarks
