#!/usr/bin/env python3
"""Run Google Benchmark sequentially and preserve its native JSON output."""
import pathlib
import subprocess
import sys

ROOT = pathlib.Path(__file__).resolve().parent
OUTPUT = ROOT / "results" / "benchmark.json"
OUTPUT.parent.mkdir(exist_ok=True)
subprocess.run(
    [str(ROOT / "build" / "coal_benchmark"),
     "--benchmark_repetitions=5",
     f"--benchmark_out={OUTPUT}",
     "--benchmark_out_format=json", *sys.argv[1:]],
    check=True,
)
