include(CheckCXXSourceRuns)

if(NOT CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|AMD64)$")
  message(
    FATAL_ERROR "The planning comparison currently targets x86-64 with AVX2/FMA"
  )
endif()
set(CMAKE_REQUIRED_FLAGS "-mavx2 -mfma")
check_cxx_source_runs(
  "int main() { return !(__builtin_cpu_supports(\"avx2\") && __builtin_cpu_supports(\"fma\")); }"
  MPB_HAS_AVX2_FMA)
unset(CMAKE_REQUIRED_FLAGS)
if(NOT MPB_HAS_AVX2_FMA)
  message(FATAL_ERROR "Planning requires an AVX2/FMA capable host")
endif()

include(cmake/Dependencies.cmake)

add_library(mpb_planning core/robot.cpp core/problem.cpp core/validation.cpp)
target_include_directories(mpb_planning PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(mpb_planning PUBLIC mpb::core)
target_compile_options(
  mpb_planning
  PUBLIC -mavx2 -mfma
  PRIVATE -Wall -Wextra -Wno-ignored-attributes -g -fno-omit-frame-pointer)
# Keep the same floating-point options for the shared FK and both adapters.
target_compile_options(
  mpb_planning PRIVATE -fno-math-errno -fno-signed-zeros -fno-trapping-math
                       -ffp-contract=fast)

add_subdirectory(backends)

add_executable(
  planning_benchmark
  benchmarks/main.cpp benchmarks/cli.cpp benchmarks/planning_benchmarks.cpp
  benchmarks/trajectory.cpp)
target_link_libraries(planning_benchmark PRIVATE mpb_planning
                                                 nlohmann_json::nlohmann_json)
target_compile_options(planning_benchmark PRIVATE -Wall -Wextra -g
                                                  -fno-omit-frame-pointer)

add_custom_target(build_planning DEPENDS planning_benchmark)

if(BUILD_TESTING)
  add_executable(planning_tests tests/planning_tests.cpp)
  target_link_libraries(planning_tests PRIVATE mpb_planning)
  add_dependencies(build_planning planning_tests)
  add_test(NAME planning_correctness COMMAND planning_tests)
endif()

set_target_properties(planning_benchmark PROPERTIES RUNTIME_OUTPUT_DIRECTORY
                                                    "${PROJECT_BINARY_DIR}")
if(BUILD_TESTING)
  set_target_properties(planning_tests PROPERTIES RUNTIME_OUTPUT_DIRECTORY
                                                  "${PROJECT_BINARY_DIR}")
endif()
