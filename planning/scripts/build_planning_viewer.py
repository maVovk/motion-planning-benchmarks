#!/usr/bin/env python3
"""Embed exported trajectories into a standalone offline animation viewer."""

import argparse
import json
import pathlib

ROOT = pathlib.Path(__file__).resolve().parents[2]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--results", type=pathlib.Path, default=ROOT / "results/planning")
args = parser.parse_args()
trajectories = [
    json.loads(path.read_text())
    for path in sorted((args.results / "trajectories").glob("*.json"))
]
if not trajectories or any(not trial["frames"] for trial in trajectories):
    raise SystemExit("No valid trajectory animations found")
template = (ROOT / "planning/visualization/planning_viewer.html").read_text()
(args.results / "viewer.html").write_text(
    template.replace(
        "__DATA__",
        json.dumps(trajectories, separators=(",", ":")).replace("</", "<\\/"),
    )
)
print(f'Embedded {len(trajectories)} trajectories in {args.results / "viewer.html"}')
