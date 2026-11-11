# Backend headers expose only benchmark types; upstream headers stay private.
target_sources(mpb_planning PRIVATE coal/checker.cpp ompl/planner.cpp)
target_link_libraries(mpb_planning PRIVATE ompl::ompl)
# The pinned OMPL target does not export its build-tree include directories.
target_include_directories(mpb_planning SYSTEM PRIVATE "${ompl_SOURCE_DIR}/src"
                                                       "${ompl_BINARY_DIR}/src")

add_subdirectory(vamp)
