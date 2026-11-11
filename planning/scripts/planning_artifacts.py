#!/usr/bin/env python3
"""Select matched successful trials and export their trajectories for inspection."""

import argparse
import json
import pathlib
import subprocess

ROOT = pathlib.Path(__file__).resolve().parents[2]


def select_case(rows, scenes, obstacle_count):
    successful = [
        {
            (row["scene_seed"], row["planner_seed"])
            for row in rows
            if row["planner"] == planner
            and row["obstacles"] == obstacle_count
            and row["success"]
        }
        for planner in ("vamp", "ompl")
    ]
    common = sorted(successful[0] & successful[1])
    if not common:
        raise RuntimeError(f"No jointly successful case for N={obstacle_count}")
    by_seed = {
        scene["scene_seed"]: scene
        for scene in scenes
        if scene["obstacles"] == obstacle_count
    }
    needing_search = [
        case for case in common if not by_seed[case[0]]["direct_path_valid"]
    ]
    return (needing_search or common)[0]


def export_trajectories(binary, output, rows, scenes):
    target = output / "trajectories"
    target.mkdir(parents=True, exist_ok=True)
    for count in (1, 10, 100):
        scene, seed = select_case(rows, scenes, count)
        for planner in ("vamp", "ompl"):
            destination = target / f"{planner}_{count}.json"
            completed = subprocess.run(
                [
                    str(binary),
                    planner,
                    str(count),
                    str(scene),
                    str(seed),
                    "--export",
                    str(destination),
                ],
                check=True,
                capture_output=True,
                text=True,
                timeout=60,
            )
            result = json.loads(completed.stdout)
            if not result["success"]:
                raise RuntimeError(
                    f"Previously successful trial failed during export: {planner}/{count}"
                )
            trajectory = json.loads(destination.read_text())
            trajectory["planner_seed"] = seed
            destination.write_text(json.dumps(trajectory, separators=(",", ":")))
            print(
                f"Exported {planner}, N={count}, scene={scene}, planner_seed={seed}",
                flush=True,
            )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--binary", type=pathlib.Path, default=ROOT / "build/planning_benchmark"
    )
    parser.add_argument(
        "--results", type=pathlib.Path, default=ROOT / "results/planning"
    )
    args = parser.parse_args()
    rows = [
        json.loads(line)
        for line in (args.results / "raw.jsonl").read_text().splitlines()
    ]
    scenes = json.loads((args.results / "scenes.json").read_text())
    export_trajectories(args.binary, args.results, rows, scenes)
