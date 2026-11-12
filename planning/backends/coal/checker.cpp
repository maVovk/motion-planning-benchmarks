#include "backends/coal/checker.hpp"

#include <coal/data_types.h>
#include <coal/math/transform.h>

#include <cstddef>
#include <memory>
#include <vector>

#include "core/robot.hpp"
#include "core/types.hpp"
#include "mpb/collision.hpp"
#include "mpb/manager.hpp"
#include "mpb/scene.hpp"

namespace mpb::planning {

namespace {

coal::Vec3s center(const Sphere& sphere) {
  return {sphere.center[0], sphere.center[1], sphere.center[2]};
}
}  // namespace

struct CoalChecker::Impl {
  std::vector<std::unique_ptr<coal::CollisionObject>> obstacles;
  std::vector<std::unique_ptr<coal::CollisionObject>> robot;
  std::unique_ptr<Manager> manager;
  CollisionCallback callback;

  explicit Impl(const std::vector<Sphere>& spheres) {
    std::vector<coal::CollisionObject*> pointers;
    for (const auto& sphere : spheres) {
      obstacles.push_back(makeSphere(center(sphere), sphere.radius));
      pointers.push_back(obstacles.back().get());
    }

    manager = buildManager(ManagerType::Tree, pointers, HashConfig{});

    for (const auto& sphere : robotSpheres(Configuration{})) {
      robot.push_back(makeSphere(center(sphere), sphere.radius));
    }
  }
};

CoalChecker::CoalChecker(const std::vector<Sphere>& obstacles)
    : impl_(std::make_unique<Impl>(obstacles)) {}

CoalChecker::~CoalChecker() = default;

bool CoalChecker::valid(const Configuration& configuration) {
  if (!withinBounds(configuration)) {
    return false;
  }

  const auto spheres = robotSpheres(configuration);
  for (size_t i = 0; i < spheres.size(); ++i) {
    impl_->robot[i]->setTranslation(center(spheres[i]));
    impl_->robot[i]->computeAABB();
  }

  for (const auto& [first, second] : selfCollisionPairs()) {
    impl_->callback.init();
    if (impl_->callback.collide(impl_->robot[first].get(),
                                impl_->robot[second].get())) {
      return false;
    }
  }

  for (const auto& object : impl_->robot) {
    impl_->callback.init();
    impl_->manager->collide(object.get(), &impl_->callback);
    if (impl_->callback.hits) {
      return false;
    }
  }

  return true;
}

}  // namespace mpb::planning
