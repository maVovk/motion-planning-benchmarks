include(FetchContent)

set(VAMP_BUILD_PYTHON_BINDINGS
    OFF
    CACHE BOOL "" FORCE)
set(VAMP_INSTALL_CPP_LIBRARY
    OFF
    CACHE BOOL "" FORCE)
set(VAMP_BUILD_TESTS
    OFF
    CACHE BOOL "" FORCE)
set(VAMP_LTO
    OFF
    CACHE BOOL "" FORCE)
FetchContent_Declare(
  vamp
  GIT_REPOSITORY https://github.com/KavrakiLab/vamp.git
  GIT_TAG 27cb9b66ebd8d101ce88d330f3b028398afc1cfa)
FetchContent_MakeAvailable(vamp)

set(OMPL_BUILD_VAMP
    OFF
    CACHE BOOL "" FORCE)
set(OMPL_BUILD_PYTHON_BINDINGS
    OFF
    CACHE BOOL "" FORCE)
set(OMPL_BUILD_DEMOS
    OFF
    CACHE BOOL "" FORCE)
set(OMPL_BUILD_TESTS
    OFF
    CACHE BOOL "" FORCE)
FetchContent_Declare(
  ompl
  GIT_REPOSITORY https://github.com/ompl/ompl.git
  GIT_TAG c509861210a63ec962bbec72c52823abc16b102e)
FetchContent_MakeAvailable(ompl)

set(JSON_BuildTests
    OFF
    CACHE INTERNAL "")
FetchContent_Declare(
  json
  GIT_REPOSITORY https://github.com/nlohmann/json.git
  GIT_TAG v3.11.3
  GIT_SHALLOW TRUE)
FetchContent_MakeAvailable(json)
