cmake_minimum_required(VERSION 3.22)
project(coal_benchmark LANGUAGES CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(POLICY CMP0167)
  cmake_policy(SET CMP0167 NEW)
endif()
find_package(coal 3 REQUIRED)
add_library(mpb_core
  src/scene.cpp
  src/manager.cpp
  src/collision.cpp
  src/memory.cpp)
add_library(mpb::core ALIAS mpb_core)
target_include_directories(mpb_core PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_compile_features(mpb_core PUBLIC cxx_std_17)
target_link_libraries(mpb_core PUBLIC coal::coal)
target_compile_options(mpb_core PRIVATE -Wall -Wextra -Wpedantic)

option(MPB_BUILD_BENCHMARKS "Build the Google Benchmark executable" ON)
if(MPB_BUILD_BENCHMARKS)
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

  add_executable(coal_benchmark
    main.cpp
    benchmarks/coal_benchmarks.cpp
    benchmarks/workloads.cpp)
  target_include_directories(coal_benchmark PRIVATE benchmarks)
  target_link_libraries(coal_benchmark PRIVATE mpb::core benchmark::benchmark)
  target_compile_options(coal_benchmark PRIVATE -Wall -Wextra -Wpedantic)
endif()
