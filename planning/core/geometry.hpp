#pragma once

#include "core/types.hpp"

namespace mpb::planning {

inline bool spheresOverlap(const Sphere& a, const Sphere& b,
                           double clearance = 0) {
  double squared = 0;
  for (size_t i = 0; i < 3; ++i) {
    const double delta = static_cast<double>(a.center[i]) - b.center[i];
    squared += delta * delta;
  }

  const double radius = static_cast<double>(a.radius) + b.radius + clearance;
  return squared <= radius * radius;
}

}  // namespace mpb::planning
