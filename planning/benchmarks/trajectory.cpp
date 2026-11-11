#include "benchmarks/trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <nlohmann/json.hpp>
#include <stdexcept>

#include "core/robot.hpp"

namespace mpb::planning {

namespace {

using Json = nlohmann::json;

Json makeFrame(const Configuration& configuration) {
  Json frame = Json::array();
  for (const auto& sphere : robotSpheres(configuration)) {
    frame.push_back(
        {sphere.center[0], sphere.center[1], sphere.center[2], sphere.radius});
  }

  return frame;
}
}  // namespace

void exportTrajectory(const std::string& filename, const Problem& problem,
                      const PlanResult& result, const std::string& planner) {
  Json output{{"planner", planner},     {"scene_seed", problem.seed},
              {"start", problem.start}, {"goal", problem.goal},
              {"path", result.path},    {"obstacles", Json::array()},
              {"frames", Json::array()}};
  for (const auto& sphere : problem.obstacles) {
    output["obstacles"].push_back(
        {{"center", sphere.center}, {"radius", sphere.radius}});
  }

  for (size_t segment = 0; segment + 1 < result.path.size(); ++segment) {
    const auto length =
        configurationDistance(result.path[segment], result.path[segment + 1]);
    const auto samples = std::max(1u, std::uint32_t(std::ceil(length / 0.05)));
    for (std::uint32_t i = 0; i < samples; ++i) {
      const auto q = interpolate(result.path[segment], result.path[segment + 1],
                                 double(i) / samples);
      output["frames"].push_back(makeFrame(q));
    }
  }

  if (!result.path.empty()) {
    output["frames"].push_back(makeFrame(result.path.back()));
  }

  std::ofstream file(filename);
  if (!file) {
    throw std::runtime_error("Cannot write trajectory: " + filename);
  }

  file << output.dump();
}

}  // namespace mpb::planning
