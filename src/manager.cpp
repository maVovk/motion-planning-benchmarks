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
#include <string>
#include <vector>

namespace mpb {
std::unique_ptr<Manager> makeManager(const std::string& name, const HashConfig& config) {
  if (name == "naive") return std::make_unique<coal::NaiveCollisionManager>();
  if (name == "sap") return std::make_unique<coal::SaPCollisionManager>();
  if (name == "ssap") return std::make_unique<coal::SSaPCollisionManager>();
  if (name == "interval") return std::make_unique<coal::IntervalTreeCollisionManager>();
  if (name == "tree") return std::make_unique<coal::DynamicAABBTreeCollisionManager>();
  if (name == "tree_array") return std::make_unique<coal::DynamicAABBTreeArrayCollisionManager>();
  if (name == "hash") {
    return std::make_unique<coal::SpatialHashingCollisionManager<>>(
        config.cell_size, coal::Vec3s::Constant(-config.half_extent),
        coal::Vec3s::Constant(config.half_extent), config.table_size);
  }
  throw std::invalid_argument("Unknown manager: " + name);
}

std::unique_ptr<Manager> buildManager(const std::string& name, const std::vector<Object*>& objects,
                                      const HashConfig& config) {
  auto manager = makeManager(name, config);
  manager->registerObjects(objects);
  manager->setup();
  return manager;
}


}  // namespace mpb
