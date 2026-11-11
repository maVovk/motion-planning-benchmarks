#!/usr/bin/env python3
"""Capture six perf recordings and generate interactive SVG flamegraphs."""

import argparse
import json
import hashlib
import os
import pathlib
import shutil
import subprocess

from planning_artifacts import select_case

ROOT = pathlib.Path(__file__).resolve().parents[2]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--binary", type=pathlib.Path, default=ROOT / "build/planning_benchmark"
    )
    parser.add_argument(
        "--perf",
        default=shutil.which("perf")
        or str(ROOT / "build/tools/perf-root/usr/lib/linux-tools-6.8.0-149/perf"),
    )
    parser.add_argument(
        "--flamegraph", type=pathlib.Path, default=ROOT / "build/tools/FlameGraph"
    )
    parser.add_argument(
        "--results", type=pathlib.Path, default=ROOT / "results/planning"
    )
    parser.add_argument("--seconds", type=float, default=3.0)
    parser.add_argument(
        "--render-only", action="store_true", help="Reuse existing perf recordings"
    )
    args = parser.parse_args()
    output = args.results / "profiles"
    output.mkdir(parents=True, exist_ok=True)
    rows = [
        json.loads(line)
        for line in (args.results / "raw.jsonl").read_text().splitlines()
    ]
    scenes = {
        (scene["obstacles"], scene["scene_seed"]): scene
        for scene in json.loads((args.results / "scenes.json").read_text())
    }
    environment = os.environ.copy()
    local_libraries = ROOT / "build/tools/perf-root/usr/lib/x86_64-linux-gnu"
    environment["LD_LIBRARY_PATH"] = (
        str(local_libraries) + ":" + environment.get("LD_LIBRARY_PATH", "")
    )
    metadata = {
        "binary_sha256": hashlib.sha256(args.binary.read_bytes()).hexdigest(),
        "event": "cpu-clock:u",
        "frequency_hz": 499,
        "stack": "dwarf,16384",
        "seconds": args.seconds,
        "scope": "entire worker process: scene generation, warmup, validation, repeated adapter setup + solve + path extraction, and reporting",
        "perf_version": subprocess.check_output(
            [args.perf, "--version"], env=environment, text=True
        ).strip(),
        "cases": [],
    }
    if args.render_only:
        # Rendering existing recordings must retain their original recording scope.
        metadata = json.loads((output / "metadata.json").read_text())
        metadata["cases"] = []
    for count in (1, 10, 100):
        scene, seed = select_case(rows, list(scenes.values()), count)
        for planner in ("vamp", "ompl"):
            stem = output / f"{planner}_{count}"
            if args.render_only:
                workload = json.loads(
                    stem.with_suffix(".log").read_text().splitlines()[0]
                )
            else:
                workload = None
            command = [
                args.perf,
                "record",
                "-e",
                "cpu-clock:u",
                "-F",
                "499",
                "--call-graph",
                "dwarf,16384",
                "-o",
                str(stem.with_suffix(".data")),
                "--",
                str(args.binary),
                planner,
                str(count),
                str(scene),
                str(seed),
                "--profile-seconds",
                str(args.seconds),
            ]
            if not args.render_only:
                recorded = subprocess.run(
                    command,
                    env=environment,
                    capture_output=True,
                    text=True,
                    timeout=args.seconds + 60,
                )
                stem.with_suffix(".log").write_text(recorded.stdout + recorded.stderr)
                recorded.check_returncode()
                workload = json.loads(recorded.stdout)
            script = subprocess.run(
                [
                    args.perf,
                    "script",
                    "-i",
                    str(stem.with_suffix(".data")),
                    "-F",
                    "comm,pid,tid,time,event,ip,sym,dso",
                ],
                env=environment,
                capture_output=True,
                check=True,
            )
            folded = subprocess.run(
                [str(args.flamegraph / "stackcollapse-perf.pl")],
                input=script.stdout,
                capture_output=True,
                check=True,
            ).stdout
            if not folded.strip():
                raise RuntimeError(f"No stack samples in {stem}")
            stem.with_suffix(".folded").write_bytes(folded)
            svg = subprocess.run(
                [
                    str(args.flamegraph / "flamegraph.pl"),
                    "--title",
                    f"{planner.upper()} RRTConnect / {count} obstacles",
                    "--countname",
                    "samples",
                ],
                input=folded,
                capture_output=True,
                check=True,
            ).stdout
            stem.with_suffix(".svg").write_bytes(svg)
            report = subprocess.run(
                [
                    args.perf,
                    "report",
                    "--stdio",
                    "--no-children",
                    "--percent-limit",
                    "1",
                    "-i",
                    str(stem.with_suffix(".data")),
                ],
                env=environment,
                capture_output=True,
                check=True,
            )
            stem.with_suffix(".txt").write_bytes(report.stdout)
            samples = sum(int(line.rsplit(b" ", 1)[1]) for line in folded.splitlines())
            metadata["cases"].append(
                dict(
                    planner=planner,
                    obstacles=count,
                    scene_seed=scene,
                    planner_seed=seed,
                    direct_path_valid=scenes[(count, scene)]["direct_path_valid"],
                    samples=samples,
                    workload=workload,
                )
            )
            print(f"Captured {planner}, N={count}: {samples} samples", flush=True)
    (output / "metadata.json").write_text(json.dumps(metadata, indent=2) + "\n")


if __name__ == "__main__":
    main()
