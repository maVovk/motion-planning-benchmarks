cmake_minimum_required(VERSION 3.22)
project(coal_benchmark LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

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
target_compile_features(mpb_core PUBLIC cxx_std_20)
target_link_libraries(mpb_core PUBLIC coal::coal)
target_compile_options(mpb_core PRIVATE -Wall -Wextra -Wpedantic)

option(MPB_BUILD_BENCHMARKS "Build the coal timing and memory benchmarks" ON)
if(MPB_BUILD_BENCHMARKS)
  add_subdirectory(benchmarks)
endif()

option(MPB_BUILD_PLANNING "Build the UR5 VAMP versus OMPL + coal comparison" OFF)
if(MPB_BUILD_PLANNING)
  include(CTest)
  add_subdirectory(planning)
endif()
