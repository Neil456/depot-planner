#!/usr/bin/env python3
"""Step 4: run the space-time battery and write results/battery_spacetime.csv."""

from __future__ import annotations

import time

from depot_planner.config import results_path
from depot_planner.eval.battery import BATTERY_CSV, run_battery, summarise, write_battery


def main() -> None:
    print("running the space-time scenario battery...")
    began = time.perf_counter()
    frame = run_battery()
    out = write_battery(frame, results_path(BATTERY_CSV))
    elapsed = time.perf_counter() - began

    summary = summarise(frame)
    with_pct = summary.copy()
    with_pct["success_pct"] = with_pct["success_pct"].round(1)
    print()
    print(with_pct[[
        "scenario", "planner", "episodes", "success_pct", "collisions", "timeouts",
        "mean_time_to_goal_s", "mean_wait_steps", "mean_planning_ms", "max_planning_ms",
    ]].to_string(index=False, float_format=lambda v: f"{v:.2f}"))
    print(f"\n{len(frame)} episodes in {elapsed:.1f}s -> {out}")


if __name__ == "__main__":
    main()
