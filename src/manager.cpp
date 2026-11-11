#include "mpb/manager.hpp"

#include "mpb/scene.hpp"

#include <coal/broadphase/broadphase_SSaP.h>
#include <coal/broadphase/broadphase_SaP.h>
#include <coal/broadphase/broadphase_bruteforce.h>
#include <coal/broadphase/broadphase_dynamic_AABB_tree.h>
#include <coal/broadphase/broadphase_dynamic_AABB_tree_array.h>
#include <coal/broadphase/broadphase_interval_tree.h>
#include <coal/broadphase/broadphase_spatialhash.h>
#include <coal/data_types.h>

#include <memory>
#include <stdexcept>
#include <vector>

namespace mpb {
std::unique_ptr<Manager> makeManager(ManagerType type, const HashConfig& config) {
  switch (type) {
    case ManagerType::Naive:
      return std::make_unique<coal::NaiveCollisionManager>();
    case ManagerType::SaP:
      return std::make_unique<coal::SaPCollisionManager>();
    case ManagerType::SSaP:
      return std::make_unique<coal::SSaPCollisionManager>();
    case ManagerType::IntervalTree:
      return std::make_unique<coal::IntervalTreeCollisionManager>();
    case ManagerType::Tree:
      return std::make_unique<coal::DynamicAABBTreeCollisionManager>();
    case ManagerType::TreeArray:
      return std::make_unique<coal::DynamicAABBTreeArrayCollisionManager>();
    case ManagerType::Hash:
      return std::make_unique<coal::SpatialHashingCollisionManager<>>(
          config.cell_size, coal::Vec3s::Constant(-config.half_extent),
          coal::Vec3s::Constant(config.half_extent), config.table_size);
  }
  throw std::invalid_argument("Unknown manager type");
}

std::unique_ptr<Manager> buildManager(ManagerType type, const std::vector<coal::CollisionObject*>& objects,
                                      const HashConfig& config) {
  auto manager = makeManager(type, config);
  manager->registerObjects(objects);
  manager->setup();
  return manager;
}


}  // namespace mpb
