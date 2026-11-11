#include "core/robot.hpp"

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <vamp/robots/ur5.hh>
#include <vamp/vector.hh>
#include <vector>

#include "core/types.hpp"
#include "ur5_pairs.hpp"

namespace mpb::planning {

std::array<Sphere, kRobotSpheres> robotSpheres(
    const Configuration& configuration) {
  using Robot = vamp::robots::UR5;
  static_assert(Robot::dimension == kJoints &&
                Robot::n_spheres == kRobotSpheres);

  Robot::ConfigurationBlock<kSimdWidth> block;
  for (size_t i = 0; i < kJoints; ++i) {
    block[i] = vamp::FloatVector<kSimdWidth>::fill(configuration[i]);
  }

  Robot::Spheres<kSimdWidth> spheres;
  Robot::sphere_fk(block, spheres);

  std::array<Sphere, kRobotSpheres> result;
  for (size_t i = 0; i < kRobotSpheres; ++i) {
    result[i] = {{spheres.x[i].to_array()[0], spheres.y[i].to_array()[0],
                  spheres.z[i].to_array()[0]},
                 spheres.r[i].to_array()[0]};
  }

  return result;
}

const std::vector<std::pair<std::uint32_t, std::uint32_t>>&
selfCollisionPairs() {
  static const std::vector<std::pair<std::uint32_t, std::uint32_t>> pairs(
      kSelfCollisionPairs.begin(), kSelfCollisionPairs.end());
  return pairs;
}

bool withinBounds(const Configuration& configuration) {
  for (size_t i = 0; i < kJoints; ++i) {
    const auto lower = vamp::robots::UR5::s_a[i];
    const auto upper = lower + vamp::robots::UR5::s_m[i];
    if (!std::isfinite(configuration[i]) || configuration[i] < lower ||
        configuration[i] > upper) {
      return false;
    }
  }

  return true;
}

}  // namespace mpb::planning
