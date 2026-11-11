find_package(benchmark CONFIG QUIET)
if(NOT benchmark_FOUND)
  include(FetchContent)
  set(BENCHMARK_ENABLE_TESTING OFF CACHE BOOL "" FORCE)
  set(BENCHMARK_ENABLE_INSTALL OFF CACHE BOOL "" FORCE)
  FetchContent_Declare(google_benchmark
    GIT_REPOSITORY https://github.com/google/benchmark.git
    GIT_TAG v1.9.4
    GIT_SHALLOW TRUE)
  FetchContent_MakeAvailable(google_benchmark)
endif()

find_package(fmt 11 CONFIG QUIET)
if(NOT fmt_FOUND)
  include(FetchContent)
  set(FMT_TEST OFF CACHE BOOL "" FORCE)
  set(FMT_DOC OFF CACHE BOOL "" FORCE)
  set(FMT_INSTALL OFF CACHE BOOL "" FORCE)
  FetchContent_Declare(fmt
    GIT_REPOSITORY https://github.com/fmtlib/fmt.git
    GIT_TAG 11.1.4
    GIT_SHALLOW TRUE)
  FetchContent_MakeAvailable(fmt)
endif()

add_executable(coal_benchmark
  main.cpp
  config.cpp
  coal_benchmarks.cpp
  memory_measurements.cpp
  workloads.cpp)
target_include_directories(coal_benchmark PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(coal_benchmark PRIVATE mpb::core benchmark::benchmark fmt::fmt)
target_compile_options(coal_benchmark PRIVATE -Wall -Wextra -Wpedantic)

add_custom_target(build_benchmarks DEPENDS coal_benchmark)

set_target_properties(coal_benchmark PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")
