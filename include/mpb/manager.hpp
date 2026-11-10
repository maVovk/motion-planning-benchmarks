#pragma once

#include "mpb/scene.hpp"

#include <coal/broadphase/broadphase_collision_manager.h>

#include <array>
#include <memory>
#include <string>
#include <vector>

namespace mpb {
using Manager = coal::BroadPhaseCollisionManager;

struct HashConfig {
  double cell_size = 0.5;
  double half_extent = 32.0;
  unsigned table_size = 10007;
};

constexpr std::array<const char*, 7> kManagers{
    "naive", "sap", "ssap", "interval", "tree", "tree_array", "hash"};

std::unique_ptr<Manager> makeManager(const std::string& name, const HashConfig& config);

// Registered objects must outlive the returned manager. Accepts arbitrary coal
// objects, so a future robot benchmark can reuse the same broad-phase factory.
std::unique_ptr<Manager> buildManager(const std::string& name,
    const std::vector<Object*>& objects, const HashConfig& config);

}  // namespace mpb
