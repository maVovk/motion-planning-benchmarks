#pragma once

#include <array>
#include <cstdint>
#include <utility>
#include <vector>

#include "core/types.hpp"

namespace mpb::planning {

// Shared model interface, implemented by backends/vamp/robot.cpp so both
// collision backends use identical UR5 geometry, pairs, and joint bounds.
std::array<Sphere, kRobotSpheres> robotSpheres(
    const Configuration& configuration);
const std::vector<std::pair<std::uint32_t, std::uint32_t>>&
selfCollisionPairs();
bool withinBounds(const Configuration& configuration);

// Model-independent joint-space operations, implemented in robot.cpp.
double configurationDistance(const Configuration& first,
                             const Configuration& second);
Configuration interpolate(const Configuration& first,
                          const Configuration& second, double fraction);

}  // namespace mpb::planning
