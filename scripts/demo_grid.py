#!/usr/bin/env python3
"""Step 1 demo: Dijkstra vs A* vs weighted A* on one depot problem.

Writes ``results/step1/compare.png``.
"""

from __future__ import annotations

import numpy as np

from depot_planner.config import load_config, results_path
from depot_planner.grid_astar.search import astar, dijkstra, weighted_astar
from depot_planner.viz.render import save_comparison, save_map_png
from depot_planner.world.depot_maps import generate_depot, sample_start_goal_pairs


def main() -> None:
    cfg = load_config("grid")
    weight = float(cfg["search"]["weight"])

    grid = generate_depot(seed=0)
    cost_map = grid.cost_map()
    min_cost = grid.min_cell_cost()
    rng = np.random.default_rng(7)
    (start, goal), = sample_start_goal_pairs(grid, 1, rng, min_separation=45.0)

    common = dict(drivable=grid.drivable, min_cell_cost=min_cost)
    results = [
        ("Dijkstra", dijkstra(cost_map, start, goal, **common)),
        ("A*", astar(cost_map, start, goal, **common)),
        (f"Weighted A* (w={weight})", weighted_astar(cost_map, start, goal, weight=weight, **common)),
    ]

    out = save_comparison(
        grid,
        results,
        start,
        goal,
        results_path("step1", "compare.png"),
        suptitle=f"Depot grid search, start {start} -> goal {goal}",
    )
    save_map_png(grid, results_path("step1", "depot_map.png"), title="Depot layout (seed 0)")

    for name, result in results:
        status = f"cost {result.cost:.3f}" if result.success else f"no path ({result.reason})"
        print(f"{name:24s} {status:24s} expanded {result.nodes_expanded:6d}  {result.runtime_ms:7.2f} ms")
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
