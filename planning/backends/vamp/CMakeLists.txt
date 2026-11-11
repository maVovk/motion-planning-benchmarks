find_package(Python3 REQUIRED COMPONENTS Interpreter)

# This source path and extractor are coupled to the VAMP pin in
# Dependencies.cmake.
set(ur5_source "${vamp_SOURCE_DIR}/src/impl/vamp/robots/ur5.hh")
set(ur5_pairs "${CMAKE_CURRENT_BINARY_DIR}/generated/ur5_pairs.hpp")
add_custom_command(
  OUTPUT "${ur5_pairs}"
  COMMAND
    "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/extract_ur5_pairs.py"
    "${ur5_source}" "${ur5_pairs}"
  DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/extract_ur5_pairs.py" "${ur5_source}"
  VERBATIM)
add_custom_target(mpb_vamp_ur5_pairs DEPENDS "${ur5_pairs}")
add_dependencies(mpb_planning mpb_vamp_ur5_pairs)

target_sources(mpb_planning PRIVATE planner.cpp robot.cpp "${ur5_pairs}")
target_include_directories(mpb_planning
                           PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/generated")
target_link_libraries(mpb_planning PRIVATE vamp::vamp)
