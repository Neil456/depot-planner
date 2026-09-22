#!/usr/bin/env python3
"""Time the Python reference against the C++ core, planner by planner.

Writes ``results/cpp_benchmark.csv``, which `scripts/make_report.py` renders as
the "Python vs C++" table in REPORT.md and `scripts/make_readme.py` as the one
in README.md. Each case is run several times on each backend and the median is
kept; scenario setup happens before the clock starts.

If the extension was not built this exits cleanly without writing anything, and
the report simply omits the section.
"""

from __future__ import annotations

import argparse
import sys

from depot_planner.config import results_path
from depot_planner.eval.cpp_bench import (
    CPP_CSV,
    DEFAULT_REPEATS,
    compare_backends,
    summarise_cpp,
    write_cpp_benchmark,
)
from depot_planner.grid_astar.backend import extension_available


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repeats", type=int, default=DEFAULT_REPEATS,
                        help="runs per case per backend; the median is kept")
    args = parser.parse_args()
    if not extension_available():
        print("the C++ core is not built; skipping the benchmark "
              "(reinstall with `pip install -e .` to build it)", file=sys.stderr)
        return 0
    print("timing the Python reference against the C++ core...")
    frame = compare_backends(repeats=args.repeats)
    out = write_cpp_benchmark(frame, results_path(CPP_CSV))
    summary = summarise_cpp(frame)
    print()
    print(summary.to_string(index=False, float_format=lambda v: f"{v:.4f}"))
    print(f"\n{len(frame)} timed cases x {args.repeats} repeats per backend -> {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
