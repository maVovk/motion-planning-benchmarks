#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace mpb::planning {

struct PlanningConfig {
  float range = 1.0F;
  static constexpr uint32_t motion_resolution = 32;
  double validation_step = 1.0 / 128.0;
  size_t max_iterations = 100000;
  size_t max_samples = 100000;
};

struct ProblemConfig {
  std::array<float, 3> lower{-0.9F, -0.9F, 0.3F};
  std::array<float, 3> upper{0.9F, 0.9F, 1.8F};
  float min_radius = 0.05F;
  float max_radius = 0.12F;
  double min_endpoint_distance = 1.5;
  uint32_t max_generation_attempts = 100000;
};

}  // namespace mpb::planning
