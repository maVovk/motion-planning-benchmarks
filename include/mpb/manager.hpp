#pragma once

#include <coal/broadphase/broadphase_collision_manager.h>

#include <array>
#include <cstdint>
#include <memory>
#include <string_view>
#include <vector>


namespace mpb {
using Manager = coal::BroadPhaseCollisionManager;

struct HashConfig {
  double cell_size = 0.5;
  double half_extent = 32.0;
  std::uint32_t table_size = 10007;
};

enum class ManagerType {
  Naive,
  SaP,
  SSaP,
  IntervalTree,
  Tree,
  TreeArray,
  Hash
};

inline constexpr std::array<std::string_view, 7> kManagerNames{
    "naive", "sap", "ssap", "interval", "tree", "tree_array", "hash"};

std::unique_ptr<Manager> makeManager(ManagerType type,
                                     const HashConfig& config);

std::unique_ptr<Manager> buildManager(ManagerType type,
                                      const std::vector<coal::CollisionObject*>& objects,
                                      const HashConfig& config);

}  // namespace mpb
