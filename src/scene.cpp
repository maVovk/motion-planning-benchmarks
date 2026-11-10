#include "mpb/scene.hpp"

#include <coal/data_types.h>
#include <coal/math/transform.h>
#include <coal/shape/geometric_shapes.h>

#include <cstddef>
#include <memory>
#include <random>

namespace mpb {
std::unique_ptr<Object> makeSphere(const coal::Vec3s& center, double radius) {
  return std::make_unique<Object>(std::make_shared<coal::Sphere>(radius),
      coal::Transform3s(coal::Matrix3s::Identity(), center));
}

Scene::Scene(std::size_t count, const SceneConfig& config) {
  objects.reserve(count);
  pointers.reserve(count);
  centers.reserve(count);
  std::mt19937 random(config.seed);
  std::uniform_real_distribution<double> coordinate(-config.half_extent, config.half_extent);
  for (std::size_t i = 0; i < count; ++i) {
    // Explicit evaluation order avoids compiler-dependent argument ordering.
    // uniform_real_distribution can still differ across standard libraries.
    const double x = coordinate(random);
    const double y = coordinate(random);
    const double z = coordinate(random);
    centers.emplace_back(x, y, z);
    objects.push_back(makeSphere(centers.back(), config.sphere_radius));
    pointers.push_back(objects.back().get());
  }
}

}  // namespace mpb
