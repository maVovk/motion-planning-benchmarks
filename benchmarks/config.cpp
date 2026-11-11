#include "config.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace mpb::benchmarks {
void validateConfig(const BenchmarkConfig& config) {
  const auto& scene = config.scene;
  const auto& hash = config.hash;
  const auto& query = config.query;

  if (!std::isfinite(scene.half_extent) || scene.half_extent <= 0 ||
      !std::isfinite(scene.sphere_radius) || scene.sphere_radius <= 0 ||
      !std::isfinite(query.outside_x) || query.queries_per_workload == 0) {
    throw std::invalid_argument("Неправильная конфигурация сцены и запросов");
  }

  if (std::abs(query.outside_x) <= scene.half_extent + 2 * scene.sphere_radius) {
    throw std::invalid_argument("outside_x должен гарантировать отсутствие коллизий со сферами из сцены");
  }

  constexpr double bounds_margin = 1e-6;
  const double required_extent =
      std::max(scene.half_extent, std::abs(query.outside_x)) +
      scene.sphere_radius + bounds_margin;

    if (!std::isfinite(hash.half_extent) || hash.half_extent < required_extent ||
      !std::isfinite(hash.cell_size) || hash.cell_size <= 0 || hash.table_size == 0) {
    throw std::invalid_argument(
        "Конфиг хеш индекса должен покрывать сцену+радиус+eps");
  }
}
}  // namespace mpb::benchmarks
