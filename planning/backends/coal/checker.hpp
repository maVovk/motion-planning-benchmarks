#pragma once

#include <memory>
#include <vector>

#include "core/types.hpp"

namespace mpb::planning {

// Owns obstacle and robot coal objects; one checker per planning thread.
class CoalChecker {
 public:
  explicit CoalChecker(const std::vector<Sphere>& obstacles);
  ~CoalChecker();
  bool valid(const Configuration& configuration);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace mpb::planning
