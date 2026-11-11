#pragma once

#include <cstdint>
#include <string>

namespace mpb::planning {

struct CommandLineOptions {
  bool describe = false;
  std::string planner;
  uint32_t obstacle_count = 0;
  uint32_t scene_seed = 0;
  uint32_t planner_seed = 0;
  std::string export_file;
  double profile_seconds = 0;
};

CommandLineOptions parseCommandLine(int argc, char** argv);

}  // namespace mpb::planning
