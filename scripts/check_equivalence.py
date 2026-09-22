#!/usr/bin/env python3
"""Compare the C++ core against the Python reference on every battery seed.

This is the wide sweep behind the C++ port's acceptance checks. The test suite
runs a fast slice of the same comparisons; this script runs all of them, which
takes minutes rather than seconds.

    python3 scripts/check_equivalence.py              # every section
    python3 scripts/check_equivalence.py --section spacetime
    python3 scripts/check_equivalence.py --episodes 5  # a quicker sweep

It exits non-zero on the first divergence it cannot explain, and prints a
summary of any equal-cost tie-break differences it can.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from typing import Any, Callable

import numpy as np

from depot_planner import core
from depot_planner.eval.battery import battery_seeds, tier_settings
from depot_planner.grid_astar.search import search
from depot_planner.spacetime.planners import make_planner
from depot_planner.world.depot_maps import generate_depot, sample_start_goal_pairs
from depot_planner.world.scenarios import SCENARIO_TYPES, generate_scenario

#: Fields two plans must agree on exactly.
PLAN_FIELDS = ("success", "cells", "times", "nodes_expanded", "wait_steps", "reason")


class Divergence(Exception):
    """A difference between the two backends that is not an equal-cost tie-break."""


def _same_cost(a: float, b: float) -> bool:
    return (math.isinf(a) and math.isinf(b)) or a == b


def _compare_plans(label: str, python_plan: Any, cpp_plan: Any, ties: list[str]) -> None:
    differing = [n for n in PLAN_FIELDS if getattr(python_plan, n) != getattr(cpp_plan, n)]
    if not differing and _same_cost(python_plan.cost, cpp_plan.cost):
        return
    if (
        python_plan.success == cpp_plan.success
        and abs(python_plan.cost - cpp_plan.cost) < 1e-9
        and differing == ["cells"]
    ):
        ties.append(label)
        return
    raise Divergence(
        f"{label}: {differing or ['cost']} differ\n"
        f"  python: ok={python_plan.success} cost={python_plan.cost!r} "
        f"nodes={python_plan.nodes_expanded} {python_plan.reason!r}\n"
        f"  cpp   : ok={cpp_plan.success} cost={cpp_plan.cost!r} "
        f"nodes={cpp_plan.nodes_expanded} {cpp_plan.reason!r}"
    )


# ----------------------------------------------------------------- sections


def check_grid(episodes: int, ties: list[str]) -> int:
    """Dijkstra, A* and weighted A* over sampled start/goal pairs."""
    grid = generate_depot(seed=0)
    cost_map, min_cost = grid.cost_map(), grid.min_cell_cost()
    rng = np.random.default_rng(31337)
    count = max(10, episodes * 7) if episodes else 200
    pairs = sample_start_goal_pairs(grid, count, rng, min_separation=15.0)
    common = dict(drivable=grid.drivable, min_cell_cost=min_cost, collect_expanded=False)
    variants = ({"heuristic": "none"}, {"heuristic": "octile"},
                {"heuristic": "octile", "weight": 1.5})
    checked = 0
    for start, goal in pairs:
        for variant in variants:
            results = {
                backend: search(cost_map, start, goal, backend=backend, **common, **variant)
                for backend in ("python", "cpp")
            }
            checked += 1
            python_result, cpp_result = results["python"], results["cpp"]
            if (python_result.path != cpp_result.path
                    or not _same_cost(python_result.cost, cpp_result.cost)
                    or python_result.nodes_expanded != cpp_result.nodes_expanded):
                raise Divergence(f"grid {variant} {start}->{goal}")
    return checked


def check_fields(episodes: int, ties: list[str]) -> int:
    """The obstacle distance field and the Dijkstra cost-to-go field."""
    from scipy.ndimage import distance_transform_edt

    from depot_planner.grid_astar.search import dijkstra_field

    checked = 0
    for seed in range(episodes or 5):
        grid = generate_depot(seed=seed)
        ours = grid.obstacle_distance(backend="cpp")
        theirs = distance_transform_edt(grid.drivable) * grid.resolution
        if not np.array_equal(ours, theirs):
            raise Divergence(f"obstacle distance field on depot seed {seed}")
        cost_map = grid.cost_map()
        rng = np.random.default_rng(seed)
        for source, _ in sample_start_goal_pairs(grid, 3, rng, min_separation=10.0):
            a = dijkstra_field(cost_map, [source], drivable=grid.drivable, backend="cpp")
            b = dijkstra_field(cost_map, [source], drivable=grid.drivable, backend="python")
            if not np.array_equal(a, b):
                raise Divergence(f"Dijkstra field from {source} on depot seed {seed}")
            checked += 1
        checked += 1
    return checked


def check_spacetime(episodes: int, ties: list[str]) -> int:
    """Both planners on every battery seed of both tiers, from three start steps."""
    checked = 0
    for tier in ("normal", "hard"):
        cfg = tier_settings(tier)
        types = list(cfg.get("scenario_types", SCENARIO_TYPES))
        count = episodes if episodes else int(cfg["episodes_per_type"])
        seeds = battery_seeds(count, int(cfg["seed_offset"]))
        for scenario_type in types:
            for seed in seeds:
                scenario = generate_scenario(scenario_type, seed)
                for planner_name in ("spacetime_astar", "baseline_replan"):
                    for step in (0, 5, 17):
                        plans = [
                            make_planner(planner_name, None, backend).plan(
                                scenario, scenario.start, step
                            )
                            for backend in ("python", "cpp")
                        ]
                        _compare_plans(
                            f"{tier}/{scenario_type} seed={seed} {planner_name} t={step}",
                            plans[0], plans[1], ties,
                        )
                        checked += 1
    return checked


def check_hybrid(episodes: int, ties: list[str]) -> int:
    """Hybrid A* on every parking battery seed of both tiers.

    Each C++ plan is also handed to the Python independent checker, which shares
    no code with either planner.
    """
    from depot_planner.eval.parking_battery import parking_tier_settings, tier_hybrid_config
    from depot_planner.hybrid.search import plan_for_scenario
    from depot_planner.sim.car_collision import check_plan
    from depot_planner.world.parking import PARKING_TYPES, generate_parking_scenario

    fields = ("success", "nodes_expanded", "direction_switches", "used_analytic",
              "collision_checks", "reason")
    checked = 0
    for tier in ("normal", "hard"):
        cfg = parking_tier_settings(tier)
        types = list(cfg.get("parking_types", PARKING_TYPES))
        count = episodes if episodes else int(cfg["episodes_per_type"])
        offset = int(cfg["seed_offset"])
        hybrid_cfg = tier_hybrid_config(tier)
        for parking_type in types:
            for index in range(count):
                seed = offset + index
                scenario = generate_parking_scenario(parking_type, seed,
                                                     hybrid_config=hybrid_cfg)
                plans = {
                    backend: plan_for_scenario(scenario, collect_explored=False,
                                               backend=backend)
                    for backend in ("python", "cpp")
                }
                python_plan, cpp_plan = plans["python"], plans["cpp"]
                label = f"{tier}/{parking_type} seed={seed}"
                differing = [n for n in fields
                             if getattr(python_plan, n) != getattr(cpp_plan, n)]
                same_cost = _same_cost(python_plan.cost, cpp_plan.cost)
                if differing or not same_cost:
                    raise Divergence(
                        f"{label}: {differing or ['cost']} differ\n"
                        f"  python: ok={python_plan.success} cost={python_plan.cost!r} "
                        f"expansions={python_plan.nodes_expanded} {python_plan.reason!r}\n"
                        f"  cpp   : ok={cpp_plan.success} cost={cpp_plan.cost!r} "
                        f"expansions={cpp_plan.nodes_expanded} {cpp_plan.reason!r}"
                    )
                if python_plan.poses != cpp_plan.poses:
                    ties.append(f"{label}: equal cost, different poses")
                if cpp_plan.success:
                    report = check_plan(scenario, cpp_plan)
                    if not report.ok:
                        raise Divergence(
                            f"{label}: the independent checker rejected the C++ plan: "
                            f"{report.describe()}"
                        )
                checked += 1
    return checked


SECTIONS: dict[str, Callable[[int, list[str]], int]] = {
    "grid": check_grid,
    "fields": check_fields,
    "spacetime": check_spacetime,
    "hybrid": check_hybrid,
}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--section", choices=sorted(SECTIONS), action="append",
                        help="run only this section (repeatable); default is all of them")
    parser.add_argument("--episodes", type=int, default=0,
                        help="seeds per scenario type; 0 means the battery's own count")
    args = parser.parse_args()

    if not core.available():
        print("the C++ core is not built; run `pip install -e .`", file=sys.stderr)
        return 2

    names = args.section or list(SECTIONS)
    ties: list[str] = []
    failed = False
    for name in names:
        began = time.perf_counter()
        try:
            checked = SECTIONS[name](args.episodes, ties)
        except Divergence as error:
            print(f"{name:10s} DIVERGED\n{error}", file=sys.stderr)
            failed = True
            continue
        print(f"{name:10s} {checked:6d} comparisons identical  "
              f"({time.perf_counter() - began:6.1f}s)")

    if ties:
        print(f"\n{len(ties)} equal-cost tie-break differences:")
        for label in ties[:20]:
            print(f"  {label}")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
