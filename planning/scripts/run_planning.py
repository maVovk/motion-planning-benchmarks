#!/usr/bin/env python3
"""Run isolated, sequential planning trials; retain failures and solve timeouts."""

import argparse
import csv
import datetime
import hashlib
import json
import os
import pathlib
import platform
import random
import signal
import statistics
import subprocess

from planning_artifacts import export_trajectories

ROOT = pathlib.Path(__file__).resolve().parents[2]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--binary", type=pathlib.Path, default=ROOT / "build/planning_benchmark"
    )
    parser.add_argument(
        "--output", type=pathlib.Path, default=ROOT / "results/planning"
    )
    parser.add_argument("--scenes", type=int, default=5)
    parser.add_argument("--trials", type=int, default=5)
    args = parser.parse_args()
    if args.scenes < 1 or args.trials < 1:
        parser.error("scenes and trials must be positive")
    args.output.mkdir(parents=True, exist_ok=True)
    jobs = [
        (planner, count, scene, seed)
        for count in (1, 10, 100)
        for scene in range(42, 42 + args.scenes)
        for seed in range(101, 101 + args.trials)
        for planner in ("vamp", "ompl")
    ]
    random.Random(2026).shuffle(jobs)
    metadata = {
        "created_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
        "platform": platform.platform(),
        "cpu": subprocess.check_output(["lscpu"], text=True),
        "binary": str(args.binary.resolve()),
        "binary_sha256": hashlib.sha256(args.binary.read_bytes()).hexdigest(),
        "compiler": subprocess.check_output(
            ["c++", "--version"], text=True
        ).splitlines()[0],
        "scene_seeds": list(range(42, 42 + args.scenes)),
        "planner_seeds": list(range(101, 101 + args.trials)),
        "shuffle_seed": 2026,
        "solve_timeout_seconds": 1.0,
        "validation_step_radians_l2": 1 / 128,
        "range_radians_l2": 1.0,
        "motion_samples_per_radian_l2": 32,
        "cpu_affinity": sorted(os.sched_getaffinity(0)),
        "vamp_commit": "27cb9b66ebd8d101ce88d330f3b028398afc1cfa",
        "ompl_version": "2.0.1",
        "coal_version": "3.0.3",
    }
    metadata["settings"] = json.loads(
        subprocess.check_output([str(args.binary), "--describe"], text=True)
    )
    (args.output / "metadata.json").write_text(json.dumps(metadata, indent=2) + "\n")
    scenes = []
    for count in (1, 10, 100):
        for scene in metadata["scene_seeds"]:
            command = [str(args.binary), "check", str(count), str(scene), "101"]
            result = subprocess.run(
                command, capture_output=True, text=True, check=True, timeout=60
            )
            scenes.append(json.loads(result.stdout))
    (args.output / "scenes.json").write_text(json.dumps(scenes, indent=2) + "\n")
    print(
        f"Checked coal/VAMP/analytic agreement on {len(scenes) * 2002} states",
        flush=True,
    )
    rows = []
    with (args.output / "raw.jsonl").open("w") as raw:
        for index, (planner, count, scene, seed) in enumerate(jobs, 1):
            command = [str(args.binary), planner, str(count), str(scene), str(seed)]
            process = subprocess.run(
                command, text=True, capture_output=True, timeout=60
            )
            if process.returncode == -signal.SIGALRM:
                row = dict(
                    planner=planner,
                    obstacles=count,
                    scene_seed=scene,
                    planner_seed=seed,
                    success=False,
                    timeout=True,
                    solve_us=1_000_000,
                    setup_us=None,
                )
            elif process.returncode:
                row = dict(
                    planner=planner,
                    obstacles=count,
                    scene_seed=scene,
                    planner_seed=seed,
                    success=False,
                    timeout=False,
                    error=process.stderr.strip(),
                    returncode=process.returncode,
                )
            else:
                row = json.loads(process.stdout)
                row["timeout"] = False
            rows.append(row)
            raw.write(json.dumps(row) + "\n")
            raw.flush()
            if index % 10 == 0:
                print(f"{index}/{len(jobs)} trials completed", flush=True)
    fields = [
        "planner",
        "obstacles",
        "trials",
        "successes",
        "timeouts",
        "no_path",
        "errors",
        "median_solve_us",
        "p90_solve_us",
        "min_solve_us",
        "max_solve_us",
        "median_setup_us",
        "median_path_length",
        "median_path_states",
    ]
    summary = []
    for count in (1, 10, 100):
        for planner in ("vamp", "ompl"):
            group = [
                row
                for row in rows
                if row["obstacles"] == count and row["planner"] == planner
            ]
            successful = [row for row in group if row["success"]]
            times = sorted(row["solve_us"] for row in successful)
            metric = lambda key: (
                statistics.median(row[key] for row in successful)
                if successful
                else None
            )
            summary.append(
                dict(
                    planner=planner,
                    obstacles=count,
                    trials=len(group),
                    successes=len(successful),
                    timeouts=sum(row["timeout"] for row in group),
                    no_path=sum(
                        not row["success"] and not row["timeout"] and "error" not in row
                        for row in group
                    ),
                    errors=sum("error" in row for row in group),
                    median_solve_us=metric("solve_us"),
                    p90_solve_us=(
                        times[max(0, (9 * len(times) + 9) // 10 - 1)] if times else None
                    ),
                    min_solve_us=times[0] if times else None,
                    max_solve_us=times[-1] if times else None,
                    median_setup_us=metric("setup_us"),
                    median_path_length=metric("path_length"),
                    median_path_states=metric("path_states"),
                )
            )
    with (args.output / "summary.csv").open("w") as file:
        writer = csv.DictWriter(file, fieldnames=fields)
        writer.writeheader()
        writer.writerows(summary)
    errors = [row for row in rows if "error" in row]
    print(
        f'Saved {len(rows)} trials; {sum(row["success"] for row in rows)} validated successes, '
        f'{sum(row["timeout"] for row in rows)} timeouts, {len(errors)} errors',
        flush=True,
    )
    if errors:
        raise SystemExit(
            "Errors are retained in raw.jsonl; investigate before reporting results"
        )
    export_trajectories(args.binary, args.output, rows, scenes)


if __name__ == "__main__":
    main()
