#pragma once

#include <coal/collision_object.h>
#include <coal/data_types.h>

#include <cstddef>
#include <memory>
#include <vector>

namespace mpb {

struct SceneConfig {
  double sphere_radius = 0.25;
  double half_extent = 10.0;
  uint32_t seed = 42;
};

std::unique_ptr<coal::CollisionObject> makeSphere(const coal::Vec3s& center, double radius);

struct Scene {
  std::vector<std::unique_ptr<coal::CollisionObject>> objects;
  Scene(size_t count, const SceneConfig& config);
  std::vector<coal::CollisionObject*> objectPointers() const;
};

}  // namespace mpb
