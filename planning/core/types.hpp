#pragma once

#include <immintrin.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace mpb::planning {

inline constexpr size_t kJoints = 6;
inline constexpr size_t kRobotSpheres = 40;
inline constexpr size_t kSimdWidth = sizeof(__m256) / sizeof(float);

using Configuration = std::array<float, kJoints>;
using Path = std::vector<Configuration>;

struct Sphere {
  std::array<float, 3> center;
  float radius;
};

struct Problem {
  uint32_t seed;
  std::vector<Sphere> obstacles;
  Configuration start;
  Configuration goal;
};

struct PlanResult {
  Path path;
  double setup_us = 0;
  double solve_us = 0;
  std::optional<size_t> iterations;  // OMPL does not provide this counter.
};

}  // namespace mpb::planning
