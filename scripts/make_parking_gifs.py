#!/usr/bin/env python3
"""Step 5: one GIF per parking scenario type, under results/gifs/parking/."""

from __future__ import annotations

from depot_planner.config import load_config, results_path
from depot_planner.hybrid.search import plan_for_scenario
from depot_planner.sim.car_collision import check_plan
from depot_planner.viz.animate import animate_parking, gif_size_mb
from depot_planner.viz.parking import save_plan_png
from depot_planner.world.parking import PARKING_TYPES, generate_parking_scenario

SEARCH_SEEDS = 8


def choose_scenario(parking_type: str, seeds: list[int]):
    """The most interesting solved scenario: the one using the most reverse."""
    best = None
    for seed in seeds:
        scenario = generate_parking_scenario(parking_type, seed)
        result = plan_for_scenario(scenario)
        if not result.success:
            continue
        score = (result.reverse_length_m, result.direction_switches)
        if best is None or score > best[0]:
            best = (score, scenario, result)
    if best is None:
        raise RuntimeError(f"no solved {parking_type} scenario in the search slice")
    return best[1], best[2]


def main() -> None:
    cfg = load_config("eval")["battery"]
    offset = int(cfg["seed_offset"])
    seeds = [offset + index for index in range(SEARCH_SEEDS)]

    for parking_type in PARKING_TYPES:
        scenario, result = choose_scenario(parking_type, seeds)
        report = check_plan(scenario, result)
        if not report.ok:
            raise AssertionError(f"{parking_type} seed {scenario.seed}: {report.describe()}")
        out = animate_parking(
            scenario, result, results_path("gifs", "parking", f"{parking_type}.gif")
        )
        save_plan_png(scenario, result, results_path("step5", f"{parking_type}_plan.png"))
        print(f"{parking_type:24s} seed {scenario.seed} layout {scenario.layout:22s} "
              f"{result.direction_switches} switches  {result.path_length_m:5.1f} m  "
              f"{gif_size_mb(out):.2f} MB -> {out.name}")


if __name__ == "__main__":
    main()
