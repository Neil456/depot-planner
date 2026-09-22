#!/usr/bin/env python3
"""Step 2 demo: one snapshot PNG per scenario type.

Writes ``results/step2/<type>.png``.
"""

from __future__ import annotations

from depot_planner.config import results_path
from depot_planner.viz.render import save_scenario_png
from depot_planner.world.scenarios import SCENARIO_TYPES, generate_scenario


def main() -> None:
    for name in SCENARIO_TYPES:
        scenario = generate_scenario(name, seed=0)
        # Snapshot roughly where the ego meets the traffic.
        t = min(len(scenario.natural_route) // 2, scenario.time_limit)
        out = save_scenario_png(scenario, results_path("step2", f"{name}.png"), t=t)
        print(f"{name:22s} agents {len(scenario.agents):2d}  limit {scenario.time_limit:3d}  -> {out}")


if __name__ == "__main__":
    main()
