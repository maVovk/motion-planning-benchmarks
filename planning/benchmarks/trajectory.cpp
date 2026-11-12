#include "benchmarks/trajectory.hpp"

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
                      const PlanResult& result, const std::string& planner,
                      uint32_t planner_seed, const Path& sampled_path) {
  Json output{{"planner", planner},
              {"scene_seed", problem.seed},
              {"planner_seed", planner_seed},
              {"start", problem.start},
              {"goal", problem.goal},
              {"path", result.path},
              {"obstacles", Json::array()},
              {"frames", Json::array()}};
  for (const auto& sphere : problem.obstacles) {
    output["obstacles"].push_back(
        {{"center", sphere.center}, {"radius", sphere.radius}});
  }

  for (const auto& q : sampled_path) {
    output["frames"].push_back(makeFrame(q));
  }

  std::ofstream file(filename);
  if (!file) {
    throw std::runtime_error("Cannot write trajectory: " + filename);
  }

  file << output.dump();
}

}  // namespace mpb::planning
