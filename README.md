# Collision and motion planning benchmarks

Part 1 benchmarks coal broad phases against a target sphere. Shared C++ helpers
are built as `mpb::core`; the timing harness uses Google Benchmark. Part 2
(UR5 RRTConnect with VAMP and OMPL + coal) is still pending.

## Build

Requires CMake 3.22+, a C++17 compiler, coal 3, and Google Benchmark. CMake uses
an installed Google Benchmark when available; otherwise it fetches v1.9.4.
A local coal installation using the pinned wheel dependencies works on Linux
x86-64 and macOS:

```sh
python3 -m venv .venv
.venv/bin/pip install -r requirements-lock.txt
PREFIX=$(.venv/bin/python -c 'import sysconfig; print(sysconfig.get_paths()["purelib"] + "/cmeel.prefix")')
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="$PREFIX"
cmake --build build -j 4
ln -sfn build/compile_commands.json compile_commands.json
```

CMake exports `build/compile_commands.json` by default. The root symlink lets
clangd find the database. AVX2 is available on the current Ryzen 5 3600 host;
this coal benchmark does not require architecture-specific compiler flags.

## Run

```sh
./build/coal_benchmark --benchmark_list_tests=true
python3 run_benchmark.py
# Short validation run across all cases:
./build/coal_benchmark --benchmark_min_time=0.001s
# Focused measurements with native JSON output:
./build/coal_benchmark --benchmark_filter='collision/tree/random/10000$' \
  --benchmark_repetitions=5 --benchmark_out=results/tree.json \
  --benchmark_out_format=json
```

The optional runner runs sequentially, requests five repetitions, forwards
additional Google Benchmark flags, and saves `results/benchmark.json`.
Google Benchmark controls iteration counts, timing, statistics and JSON/CSV
export. Repetitions use the same deterministic scene (seed 42), rather than
varying seeds. Configure scenes and spatial hashing in `BenchmarkConfig`,
`SceneConfig`, `QueryConfig`, and `HashConfig` in the files below.

## Code organization and reuse

* `include/mpb/scene.hpp`, `src/scene.cpp`: seeded sphere scenes, object ownership,
  and sphere construction.
* `include/mpb/manager.hpp`, `src/manager.cpp`: configuration and factories for
  the seven broad phases. `buildManager` accepts a vector of arbitrary coal
  object pointers, so it can also index robot or environment geometry.
* `include/mpb/collision.hpp`, `src/collision.cpp`: narrow-phase callback,
  collision/candidate counts, and optional early exit.
* `include/mpb/memory.hpp`, `src/memory.cpp`: Linux/macOS memory snapshots.
* `benchmarks/config.hpp`: benchmark sizes and query configuration.
* `benchmarks/workloads.*`: sphere query generation and analytic validation.
* `benchmarks/coal_benchmarks.*`: Google Benchmark timing and registration.
* `main.cpp`: command-line handling and benchmark startup.

Future planning executables can link `mpb::core` without Google Benchmark.
Configure with `-DMPB_BUILD_BENCHMARKS=OFF` to build just the shared library.
A scene owns its objects; a manager only borrows their pointers and must be
destroyed before those objects. When moving objects, update their transforms
and AABBs, then update any manager that indexes them. The current measurements
use static obstacles. Robot kinematics, self-collision filtering, and planner
state validation still need implementing for part 2; the sphere-only oracle
stays in the benchmark module.

## What this measures and why

Coal is a C++ collision-detection and distance library, formerly HPP-FCL,
derived from FCL ([upstream](https://github.com/coal-library/coal)). Here its
broad phase cheaply selects candidate obstacle pairs using bounding boxes;
its narrow phase checks the actual sphere geometry. A broad-phase candidate
is not necessarily a collision, so early exit happens only after narrow-phase
confirmation. Comparing construction time, retained memory, and query cost
shows the tradeoff between cheap setup and faster repeated checks, which
matters when a motion planner checks many robot configurations.

A small visualization would be useful for checking scene bounds, target
positions, and hit/miss behavior. Show obstacle spheres, one selected target,
and optionally its AABB and candidate obstacles; a 10000-sphere scene needs
transparency or filtering to stay readable. Export exact scene coordinates
and query results outside the timed region, so rendering does not affect
measurements. This would be a debugging aid rather than performance evidence.
For part 2, animation is more valuable: replay the UR5 path through the same
obstacles to inspect motion and clearance. Build that viewer around exported
scenes and trajectories so both planner implementations can share it.

## Measurements

All seven discrete managers from `coal/broadphase/broadphase.h` are covered:
Naive, SaP, SSaP, IntervalTree, DynamicAABBTree, DynamicAABBTreeArray and
SpatialHashing. Each uses 1, 10, 100, 1000 and 10000 obstacle spheres.
Every sphere has radius 0.25. Obstacle centers are uniform in [-10,10]^3.
Spatial hashing uses cell size 0.5, bounds [-32,32]^3 and table size 10007.

* `scene/create_destroy/N`: scene construction and destruction, including
  geometry, collision objects, AABBs and storage vectors.
* `initialize/MANAGER/N`: manager constructor, registration and setup on an
  existing scene. Manager destruction is excluded using paused timing.
  Setup outside the benchmark iteration loop is excluded by Google Benchmark.
* `collision/MANAGER/WORKLOAD/N`: a batch of 128 precomputed targets, including
  callback reset, broad phase and coal narrow phase, stopping at the first
  actual collision. Target creation and AABB preparation are excluded.
  **Reported time is per batch**; divide by 128 for time per query.
  `items_per_second` reports queries/second. `candidates_per_query` and
  `hit_fraction` describe the workload.

Workloads are `outside_miss` (target x=30), `hit` (centers of the first up to
128 obstacles, cycling for smaller scenes), and `random` (independently seeded
uniform target centers). Before collision timing, every query is checked
against an analytic sphere-distance oracle in both all-hits and early-exit
modes. These checks also warm the index and narrow phase.

Initialization benchmarks expose `scene_heap_bytes`, `index_heap_bytes`,
`scene_rss_bytes`, and `index_rss_bytes` from a separate, untimed memory probe.
On Linux, heap is glibc `mallinfo2().uordblks + hblkhd`, and RSS comes from
`/proc/self/statm`. On macOS, heap uses `malloc_zone_statistics` and RSS uses
`task_info`. Scene heap includes vector storage and separate sphere geometries.
Deltas are signed retained process allocations, not peak allocation counts.
Allocator reuse, lazy initialization and page rounding can affect snapshots;
RSS deltas may be zero or negative. Query allocation peaks are not measured.

Measurements use warmed static scenes, without index updates. There is no
CPU affinity or frequency control. Tiny initialization times include timing
pause/resume overhead effects; repeat runs on a quiet host before drawing
performance conclusions. Results depend on scene density and query order.

`results/REPORT.md`, `results/raw.csv`, and `results/summary.csv` are historical
macOS measurements from the previous custom harness. Their methodology differs
from the Google Benchmark implementation; they are preserved as historical data.

References: [coal](https://github.com/coal-library/coal),
[Google Benchmark](https://github.com/google/benchmark).
