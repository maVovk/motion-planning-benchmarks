# UR5: VAMP RRTConnect versus OMPL RRTConnect + coal

This compares two complete planning implementations on reproducible sphere scenes.
It does not isolate AVX2's contribution. See [measured results](../results/planning/REPORT.md)
and open [the offline animation](../results/planning/viewer.html) in a browser.

## Build and run

Use the same local coal prefix as part 1. Planning is opt-in, requires Linux
(POSIX solve deadlines), and currently targets x86-64 AVX2/FMA.
CMake checks the host CPU, fetches pinned VAMP/OMPL sources and their dependencies,
and disables their Python bindings and demo builds.
Run the commands below from the repository root. To build this part alone,
also pass `-DMPB_BUILD_BENCHMARKS=OFF`; only `planning/`, the shared `include/`
and `src/` folders, the root build file, and the dependency lock file are needed.

```sh
PREFIX=$(.venv/bin/python -c 'import sysconfig; print(sysconfig.get_paths()["purelib"] + "/cmeel.prefix")')
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH="$PREFIX" -DMPB_BUILD_PLANNING=ON
cmake --build build --target build_planning -j 4
ctest --test-dir build -R '^planning_correctness$' --output-on-failure
./build/planning_benchmark --describe
./build/planning_benchmark check 100 42 101
./build/planning_benchmark vamp 100 42 101 --export /tmp/vamp_path.json
./build/planning_benchmark ompl 100 42 101 --export /tmp/ompl_path.json
# Fresh process per trial, deterministic shuffled order, sequential timing:
taskset -c 2 python3 planning/scripts/run_planning.py
python3 planning/scripts/build_planning_viewer.py
```

The runner defaults to five scene seeds (42–46) and five planner seeds (101–105)
for each of 1, 10, 100 obstacles and both planners: 150 trials. Change experiment
counts with `--scenes` and `--trials`, and use `--output` for a separate run.
`PlanningConfig` and `ProblemConfig` in `core/config.hpp` define the model-independent
parameters. Both adapters report setup and solve time separately. Time is measured
using `steady_clock`, not Google Benchmark's adaptive repetitions: planning trials
are stochastic, can fail, and need a fresh seeded planner plus an individual budget.
Part 1 continues to use Google Benchmark.

## Geometry and checking

The pinned VAMP UR5 model has six joints and 40 spheres, including its gripper.
Its base sphere is at world z=0.9144 m; this is the supplied model's frame, not
an offset introduced by the experiment. Both backends use VAMP's generated sphere
forward kinematics. Sharing FK ensures the comparison uses identical positions;
it also means this is not an independent scalar-kinematics baseline.

`planning/backends/vamp/extract_ur5_pairs.py` extracts the 383 checked self-collision pairs from
VAMP's debug routine during the build. Pairs omitted by VAMP, including deliberate
adjacent-link overlaps, are omitted by coal too. The coal adapter uses part 1's
sphere factory, manager factory, and collision callback. Its static obstacle index
is a DynamicAABBTree; robot spheres are moved and have their AABBs recomputed for
each validity query. Self-collision pairs go directly through coal narrow phase.

Joints use the VAMP float bounds approximately [-pi, pi], with a Euclidean joint
metric and no angle wrapping. Both planners use range 1 rad in joint-space L2.
The collision sampling density is 32 samples/rad, rounded to multiples of eight.
OMPL checks a uniform grid in increasing interpolation order; VAMP checks the same
nominal grid in interleaved SIMD batches. Floating-point precision and early-exit
order can differ. This is discrete motion checking, not continuous collision
checking or a proof of clearance between samples.

Before timing, the runner uses the `check` command to verify 2,000 random configurations plus both endpoints
in every scene against VAMP, coal, and a double-arithmetic sphere-distance oracle.
Every returned path must start/end at the requested configurations, stay in joint
bounds, and pass coal plus the analytic oracle at <=1/128 rad L2 spacing. These
checks and their allocations are outside the reported solve time. Both checkers
share FK, so they cannot independently detect a bug in that generated FK code.

## Random problems and timing boundaries

Start and goal configurations are sampled uniformly and rejected for self-collision;
they must be separated by at least 1.5 rad L2. Obstacle centers are uniform in
[-0.9,0.9] x [-0.9,0.9] x [0.3,1.8] metres and radii in [0.05,0.12] metres.
Obstacles intersecting either endpoint robot, with a 5 mm margin, are rejected.
Obstacle–obstacle overlap is allowed. Scenes are conditioned on valid endpoints,
never on planner success or on whether the direct path is blocked. All requested
scene seeds are retained, including hard or possibly disconnected problems.
Saved scene coordinates and endpoints are in `results/planning/scenes.json`.

Each trial gets a hard one-second solve deadline. Because VAMP has no wall-clock
cancellation hook, a POSIX real-time timer terminates the isolated worker with
SIGALRM. The runner records a timeout as a right-censored one-second observation,
not a successful solve. VAMP additionally retains its native 100,000 iteration
and sample caps and can report failure before the time limit. OMPL also gets its
native time termination condition; approximate OMPL solutions are not accepted.
The runner's separate 60-second process guard catches hangs outside solving.

VAMP keeps its native dynamic-domain and tree-balancing settings, uses its seeded
SIMD XORShift sampler, and first tries a direct start-to-goal connection. OMPL uses
its standard RRTConnect tree growth and native uniform sampler with explicit local
seeds. Equal seed numbers do not imply identical sample sequences. Neither path
is simplified or optimized. Path lengths use Euclidean joint distance in radians;
there is no velocity, acceleration or execution-time model.

Setup includes the collision environment/index and planner/sampler preparation.
Solve includes the actual library solve call, including allocations and cleanup
inside that call. External result conversion, validation, export and environment
teardown are excluded from solve timing. A first allocation can influence the
fresh-process solve latency; VAMP's own internal timer omits some allocations,
so this comparison deliberately measures the external call instead.

Summary latency and path-length statistics are **conditioned on validated
successes**, with failures and timeouts shown separately. Do not compare these
medians without considering success rates. No run is discarded for being slow.
Native-library defaults, nearest-neighbor structures, arithmetic, early exits,
and the direct-path shortcut all contribute to observed differences.

## perf and flamegraphs

Install a working `perf` for the host and obtain the standard FlameGraph scripts:

```sh
git clone https://github.com/brendangregg/FlameGraph.git build/tools/FlameGraph
# This is the FlameGraph revision used for the saved results:
git -C build/tools/FlameGraph checkout 41fee1f99f9276008b7cd112fca19dc3ea84ac32
taskset -c 2 python3 planning/scripts/profile_planning.py --perf /path/to/perf
```

`profile_planning.py` defaults to a local Ubuntu perf binary if no system `perf`
is found. On this WSL2 host it was extracted from Ubuntu's
`linux-tools-6.8.0-149` package; `libtraceevent1` was extracted alongside it.
No kernel settings or system packages were changed. Other machines should use
their own working perf installation. Availability of hardware events varies;
these profiles use the software `cpu-clock:u` event at 499 Hz with 16 KiB DWARF
stack dumps.

For each obstacle count, select the first scene/seed pair both planners solved,
preferring a scene whose direct path is blocked. Both backends are profiled on
that same pair. The script runs `perf record` around the entire worker process.
The worker validates a warmup solve and then repeats fresh adapter setup, solve,
and path extraction for three seconds so fast VAMP solves produce enough samples.
Recordings also include scene generation, warmup, path validation, and reporting;
metadata records this scope. Recording control stays in the script.

To record one case manually:

```sh
perf record -e cpu-clock:u -F 499 --call-graph dwarf,16384 -o /tmp/vamp.data -- \
  ./build/planning_benchmark vamp 100 42 101 --profile-seconds 3
```

The saved profiles in `results/planning/` predate this change and used FIFO controls
to isolate the repetition loop. Regenerated profiles use the whole-process scope
above, so their sample percentages may differ.

The six SVGs are interactive: click a stack to zoom and use Search to find symbols.
Folded stacks, text reports, sample counts, workload counts and recording logs are
saved beside them. Raw `.data` files are kept locally but ignored by Git because
of their size. Regenerate them with the command above. The figures describe one
selected problem per size, not an average over all scenes. Repeated profile calls
reuse warmed code and allocator pages, while latency trials use fresh processes;
profile throughput should not be substituted for the latency measurements. Some
libc leaf symbols lack debug names and appear as addresses.

## Code map

* `core/config.hpp`, `core/types.hpp`: configuration and common data types.
* `core/problem.*`: seeded problem generation.
* `core/robot.*`: shared robot interface, interpolation, and joint-space distance.
* `core/validation.*`: checker agreement and dense path validation.
* `benchmarks/main.cpp`, `benchmarks/cli.*`: entry point and command-line parsing.
* `benchmarks/planning_benchmarks.*`: trial execution, checks, profiling, and reporting.
* `benchmarks/trajectory.*`: trajectory and robot-frame JSON export.
* `runtime/deadline.hpp`: worker solve deadline.
* `backends/coal/checker.*`: coal environment and self-collision validity checker.
* `backends/ompl/planner.*`: `OmplPlanner`, seeded sampling and coal motion validation.
* `backends/vamp/planner.*`: `VampPlanner`, VAMP solving and validity checking.
* `backends/vamp/robot.cpp`: shared UR5 model implemented through VAMP.
* `backends/vamp/extract_ur5_pairs.py`, `backends/vamp/CMakeLists.txt`: private build-time
  extraction of checked pairs; see [VAMP maintenance notes](backends/vamp/README.md).
* `cmake/Dependencies.cmake`: pinned upstream dependencies and build options.
* `scripts/run_planning.py`: trial orchestration and native JSONL/CSV results.
* `tests/planning_tests.cpp`: checker and path correctness tests.
* `visualization/planning_viewer.html`: offline canvas viewer template.

VAMP and OMPL are fetched and linked as upstream libraries; their planner
implementations are not copied here. Both adapters expose an obstacle constructor,
`valid(configuration)`, and `solve(problem, config, seed)`. The benchmark uses one
shared helper to include environment construction in setup time for either adapter.
Adapter headers expose the common types from `core/types.hpp`, keeping upstream
includes inside the backend implementations.
The VAMP model and pair extraction are shared deliberately so both collision
backends check the same geometry.

The viewer shows the exact collision spheres, allows orbiting, zooming, pausing
and scrubbing, and contains all trajectory data in one HTML file. It visualizes
joint interpolation, not physically timed robot execution. Its six representative
successful trials use matched scene/seed pairs, selected by the same rule as the
profiles; labels show the scene seed. Re-export them without rerunning timings
with `python3 planning/scripts/planning_artifacts.py`.

Sources: [VAMP](https://github.com/KavrakiLab/vamp),
[OMPL RRTConnect](https://ompl.kavrakilab.org/classompl_1_1geometric_1_1RRTConnect.html),
[coal](https://github.com/coal-library/coal),
[FlameGraph](https://github.com/brendangregg/FlameGraph).
