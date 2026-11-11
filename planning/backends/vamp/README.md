# VAMP integration

`planner.*` wraps the upstream RRTC solver. `robot.cpp` implements the common
`core/robot.hpp` model interface using VAMP's UR5 forward kinematics, joint bounds,
and checked self-collision pairs. Both collision backends intentionally use this
same robot model. Joint distance and interpolation remain in `planning/core/robot.cpp`.

`extract_ur5_pairs.py` is a build-time compatibility adapter for the pinned VAMP
revision in `planning/cmake/Dependencies.cmake`. VAMP does not expose a pair table
through the interface used here, so the script reads the explicit pair indices
in `UR5::fkcc_debug`. The local `CMakeLists.txt` owns the upstream source path,
generation rule, and private generated-header directory. Generated data stays
in the build tree; experiment scripts do not depend on VAMP's source layout.

When updating VAMP:

1. Review `src/impl/vamp/robots/ur5.hh`, especially `fkcc_debug` and `fkcc`, and
   confirm the extraction still describes the pairs checked by the solver.
2. Rebuild `build_planning` to regenerate the header. The current model has six
   joints, 40 spheres, and 383 checked pairs; review changes to those assumptions.
3. Run `ctest --test-dir build -R '^planning_correctness$' --output-on-failure`
   to compare VAMP, coal, and the analytic checker and validate both solver paths.
