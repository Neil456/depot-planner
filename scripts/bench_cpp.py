#!/usr/bin/env python3
"""Step 7: time the Python step-1 search against the C++ core.

Writes ``results/cpp_benchmark.csv``, which `scripts/make_report.py` renders as
the "Python vs C++ runtime" table in REPORT.md. If the extension was not built
this exits cleanly without writing anything, and the report simply omits the
section.
"""

from __future__ import annotations

import sys

from depot_planner.config import results_path
from depot_planner.eval.cpp_bench import CPP_CSV, compare_backends, summarise_cpp, write_cpp_benchmark
from depot_planner.grid_astar.backend import extension_available


def main() -> int:
    if not extension_available():
        print("the C++ core is not built; skipping the benchmark "
              "(reinstall with `pip install -e .` to build it)", file=sys.stderr)
        return 0
    print("timing the Python search against the C++ core...")
    frame = compare_backends()
    out = write_cpp_benchmark(frame, results_path(CPP_CSV))
    summary = summarise_cpp(frame)
    print()
    print(summary.to_string(index=False, float_format=lambda v: f"{v:.4f}"))
    print(f"\n{len(frame)} timed comparisons -> {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
