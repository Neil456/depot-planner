#!/usr/bin/env python3
"""Step 5: run the parking battery and write results/battery_parking.csv."""

from __future__ import annotations

import time

from depot_planner.config import results_path
from depot_planner.eval.parking_battery import (
    PARKING_CSV,
    run_parking_battery,
    summarise_parking,
    write_parking_battery,
)


def main() -> None:
    print("running the hybrid A* parking battery...")
    began = time.perf_counter()
    frame = run_parking_battery()
    out = write_parking_battery(frame, results_path(PARKING_CSV))
    elapsed = time.perf_counter() - began

    summary = summarise_parking(frame)
    print()
    print(summary.to_string(index=False, float_format=lambda v: f"{v:.2f}"))
    print(f"\n{len(frame)} scenarios in {elapsed:.1f}s -> {out}")


if __name__ == "__main__":
    main()
