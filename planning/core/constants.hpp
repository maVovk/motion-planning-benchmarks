#pragma once

#include <numbers>

namespace mpb::planning {

inline constexpr double PI = std::numbers::pi_v<float>;
inline constexpr double kEndpointTolerance = 1e-4;

}  // namespace mpb::planning
