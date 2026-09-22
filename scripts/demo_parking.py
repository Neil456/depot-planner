#!/usr/bin/env python3
"""Step 5 demo: one PNG per parking scenario type, in results/step5/."""

from __future__ import annotations

from depot_planner.config import results_path
from depot_planner.viz.parking import save_scenario_png
from depot_planner.world.parking import PARKING_TYPES, generate_parking_scenario


def main() -> None:
    for name in PARKING_TYPES:
        scenario = generate_parking_scenario(name, seed=0)
        out = save_scenario_png(scenario, results_path("step5", f"{name}_layout.png"))
        print(f"{name:24s} {scenario.layout:22s} -> {out}")


if __name__ == "__main__":
    main()
