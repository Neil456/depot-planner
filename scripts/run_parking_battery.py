#!/usr/bin/env python3
"""Run the hybrid A* parking battery (both tiers).

Writes ``results/battery_parking.csv`` (normal tier) and
``results/battery_parking_hard.csv`` (hard tier).
"""

from __future__ import annotations

import argparse
import time

from depot_planner.config import results_path
from depot_planner.eval.parking_battery import (
    TIERS,
    parking_tier_csv,
    parking_tier_settings,
    run_parking_battery,
    summarise_parking,
    tier_hybrid_config,
    write_parking_battery,
)


def run_tier(tier: str) -> None:
    settings = parking_tier_settings(tier)
    cap = tier_hybrid_config(tier)["limits"]["max_expansions"]
    print(f"\n=== {tier} tier ===")
    print(f"types: {', '.join(settings.get('parking_types', ['(all normal types)']))}")
    print(f"scenarios per type: {settings['episodes_per_type']}, seed offset "
          f"{settings['seed_offset']}, expansion cap {cap}")
    began = time.perf_counter()
    frame = run_parking_battery(tier=tier)
    out = write_parking_battery(frame, results_path(parking_tier_csv(tier)))
    print()
    print(summarise_parking(frame).to_string(index=False, float_format=lambda v: f"{v:.2f}"))
    print(f"\n{len(frame)} scenarios in {time.perf_counter() - began:.1f}s -> {out}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--tier", choices=(*TIERS, "all"), default="all")
    args = parser.parse_args()
    for tier in (TIERS if args.tier == "all" else (args.tier,)):
        run_tier(tier)


if __name__ == "__main__":
    main()
