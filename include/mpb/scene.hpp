#pragma once

#include <coal/collision_object.h>
#include <coal/data_types.h>

#include <cstddef>
#include <memory>
#include <vector>

namespace mpb {
using Object = coal::CollisionObject;

struct SceneConfig {
  double sphere_radius = 0.25;
  double half_extent = 10.0;
  unsigned seed = 42;
};


std::unique_ptr<Object> makeSphere(const coal::Vec3s& center, double radius);

// Owns the collision objects; pointers is a non-owning view for coal managers.
// Keep this scene alive and its object collection unchanged while a manager uses it.
struct Scene {
  std::vector<std::unique_ptr<Object>> objects;
  std::vector<Object*> pointers;
  std::vector<coal::Vec3s> centers;

  Scene(std::size_t count, const SceneConfig& config);
};

}  // namespace mpb
