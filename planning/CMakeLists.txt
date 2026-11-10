include(FetchContent)
include(CheckCXXSourceRuns)

if(NOT CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|AMD64)$")
  message(FATAL_ERROR "The planning comparison currently targets x86-64 with AVX2/FMA")
endif()
set(CMAKE_REQUIRED_FLAGS "-mavx2 -mfma")
check_cxx_source_runs("int main() { return !(__builtin_cpu_supports(\"avx2\") && __builtin_cpu_supports(\"fma\")); }" MPB_HAS_AVX2_FMA)
unset(CMAKE_REQUIRED_FLAGS)
if(NOT MPB_HAS_AVX2_FMA)
  message(FATAL_ERROR "Planning requires an AVX2/FMA capable host")
endif()

set(VAMP_BUILD_PYTHON_BINDINGS OFF CACHE BOOL "" FORCE)
set(VAMP_INSTALL_CPP_LIBRARY OFF CACHE BOOL "" FORCE)
set(VAMP_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(VAMP_LTO OFF CACHE BOOL "" FORCE)
FetchContent_Declare(vamp
  GIT_REPOSITORY https://github.com/KavrakiLab/vamp.git
  GIT_TAG 27cb9b66ebd8d101ce88d330f3b028398afc1cfa)
FetchContent_MakeAvailable(vamp)

set(OMPL_BUILD_VAMP OFF CACHE BOOL "" FORCE)
set(OMPL_BUILD_PYTHON_BINDINGS OFF CACHE BOOL "" FORCE)
set(OMPL_BUILD_DEMOS OFF CACHE BOOL "" FORCE)
set(OMPL_BUILD_TESTS OFF CACHE BOOL "" FORCE)
FetchContent_Declare(ompl
  GIT_REPOSITORY https://github.com/ompl/ompl.git
  GIT_TAG c509861210a63ec962bbec72c52823abc16b102e)
FetchContent_MakeAvailable(ompl)

set(JSON_BuildTests OFF CACHE INTERNAL "")
FetchContent_Declare(json
  GIT_REPOSITORY https://github.com/nlohmann/json.git
  GIT_TAG v3.11.3
  GIT_SHALLOW TRUE)
FetchContent_MakeAvailable(json)

find_package(Python3 REQUIRED COMPONENTS Interpreter)
set(UR5_PAIRS "${CMAKE_CURRENT_BINARY_DIR}/generated/ur5_pairs.hpp")
add_custom_command(OUTPUT "${UR5_PAIRS}"
  COMMAND "${Python3_EXECUTABLE}" "${PROJECT_SOURCE_DIR}/scripts/extract_ur5_pairs.py"
    "${vamp_SOURCE_DIR}/src/impl/vamp/robots/ur5.hh" "${UR5_PAIRS}"
  DEPENDS "${PROJECT_SOURCE_DIR}/scripts/extract_ur5_pairs.py" "${vamp_SOURCE_DIR}/src/impl/vamp/robots/ur5.hh"
  VERBATIM)

add_library(mpb_planning
  robot.cpp
  problem.cpp
  coal_checker.cpp
  vamp_planner.cpp
  ompl_planner.cpp
  validation.cpp
  "${UR5_PAIRS}")
target_include_directories(mpb_planning PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}" PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/generated")
target_link_libraries(mpb_planning PUBLIC mpb::core vamp::vamp nlohmann_json::nlohmann_json PRIVATE ompl)
target_include_directories(mpb_planning SYSTEM PRIVATE "${ompl_SOURCE_DIR}/src" "${ompl_BINARY_DIR}/src")
target_compile_options(mpb_planning PUBLIC -mavx2 -mfma PRIVATE -Wall -Wextra -Wno-ignored-attributes -g -fno-omit-frame-pointer)
# Keep the same floating-point options for the shared FK and both adapters.
target_compile_options(mpb_planning PRIVATE -fno-math-errno -fno-signed-zeros -fno-trapping-math -ffp-contract=fast)

add_executable(planning_benchmark main.cpp)
target_link_libraries(planning_benchmark PRIVATE mpb_planning)
target_compile_options(planning_benchmark PRIVATE -Wall -Wextra -g -fno-omit-frame-pointer)

add_custom_target(build_planning DEPENDS planning_benchmark)

if(BUILD_TESTING)
  add_executable(planning_tests ../tests/planning_tests.cpp)
  target_link_libraries(planning_tests PRIVATE mpb_planning)
  add_dependencies(build_planning planning_tests)
  add_test(NAME planning_correctness COMMAND planning_tests)
endif()

set_target_properties(planning_benchmark PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")
if(BUILD_TESTING)
  set_target_properties(planning_tests PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")
endif()
