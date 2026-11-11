#include "mpb/scene.hpp"

#include <coal/data_types.h>
#include <coal/math/transform.h>
#include <coal/shape/geometric_shapes.h>

#include <cstddef>
#include <memory>
#include <random>

namespace mpb {
std::unique_ptr<coal::CollisionObject> makeSphere(const coal::Vec3s& center,
                                                  double radius) {
  return std::make_unique<coal::CollisionObject>(
      std::make_shared<coal::Sphere>(radius),
      coal::Transform3s(coal::Matrix3s::Identity(), center));
}

Scene::Scene(size_t count, const SceneConfig& config) {
  objects.reserve(count);
  std::mt19937 random(config.seed);
  std::uniform_real_distribution<double> coordinate(-config.half_extent,
                                                    config.half_extent);

  for (size_t i = 0; i < count; ++i) {
    const double x = coordinate(random);
    const double y = coordinate(random);
    const double z = coordinate(random);

    objects.push_back(makeSphere(coal::Vec3s(x, y, z), config.sphere_radius));
  }
}

std::vector<coal::CollisionObject*> Scene::objectPointers() const {
  std::vector<coal::CollisionObject*> pointers;
  pointers.reserve(objects.size());
  for (const auto& object : objects) {
    pointers.push_back(object.get());
  }
  return pointers;
}

}  // namespace mpb
