#!/usr/bin/env python3
"""Run the space-time battery (both tiers).

Writes ``results/battery_spacetime.csv`` (normal tier) and
``results/battery_spacetime_hard.csv`` (hard tier).
"""

from __future__ import annotations

import argparse
import time

from depot_planner.config import results_path
from depot_planner.eval.battery import (
    TIERS,
    run_battery,
    summarise,
    tier_csv,
    tier_planner_config,
    tier_settings,
    write_battery,
)

COLUMNS = [
    "scenario", "planner", "episodes", "success_pct", "collisions", "timeouts",
    "mean_time_to_goal_s", "mean_wait_steps", "mean_planning_ms", "max_planning_ms",
]


def run_tier(tier: str) -> None:
    settings = tier_settings(tier)
    planner_config = tier_planner_config(tier)
    budget = None if planner_config is None else planner_config["spacetime"]["time_limit_ms"]
    print(f"\n=== {tier} tier ===")
    print(f"scenarios: {', '.join(settings.get('scenario_types', ['(all normal types)']))}")
    print(f"episodes per type: {settings['episodes_per_type']}, seed offset {settings['seed_offset']}"
          + (f", planning budget {budget:.0f} ms/replan" if budget else ""))
    began = time.perf_counter()
    frame = run_battery(tier=tier)
    out = write_battery(frame, results_path(tier_csv(tier)))
    summary = summarise(frame)
    print()
    print(summary[COLUMNS].to_string(index=False, float_format=lambda v: f"{v:.2f}"))
    print(f"\n{len(frame)} episodes in {time.perf_counter() - began:.1f}s -> {out}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--tier", choices=(*TIERS, "all"), default="all")
    args = parser.parse_args()
    for tier in (TIERS if args.tier == "all" else (args.tier,)):
        run_tier(tier)


if __name__ == "__main__":
    main()
