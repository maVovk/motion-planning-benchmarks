#include "core/robot.hpp"

#include <cmath>
#include <cstddef>

namespace mpb::planning {

double configurationDistance(const Configuration& first,
                             const Configuration& second) {
  double squared = 0;
  for (size_t i = 0; i < kJoints; ++i) {
    const double difference = double(first[i]) - second[i];
    squared += difference * difference;
  }

  return std::sqrt(squared);
}

Configuration interpolate(const Configuration& first,
                          const Configuration& second, double fraction) {
  Configuration result;
  for (size_t i = 0; i < kJoints; ++i) {
    result[i] = first[i] + fraction * (double(second[i]) - first[i]);
  }

  return result;
}

}  // namespace mpb::planning
