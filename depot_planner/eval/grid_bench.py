"""Step-1 benchmark: Dijkstra vs A* vs weighted A* over many start/goal pairs."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd

from depot_planner.config import load_config, results_path
from depot_planner.grid_astar.search import astar, dijkstra, weighted_astar
from depot_planner.world.depot_maps import generate_depot, sample_start_goal_pairs

GRID_CSV = "grid_benchmark.csv"


def run_grid_benchmark(
    pairs: int | None = None,
    config: dict[str, Any] | None = None,
) -> pd.DataFrame:
    """Run all three algorithms on the same start/goal pairs; one row per run."""
    cfg = (config if config is not None else load_config("eval"))["grid_benchmark"]
    grid_cfg = load_config("grid")
    weight = float(grid_cfg["search"]["weight"])
    count = int(pairs if pairs is not None else cfg["pairs"])

    grid = generate_depot(seed=int(cfg["map_seed"]))
    cost_map = grid.cost_map()
    min_cost = grid.min_cell_cost()
    rng = np.random.default_rng(int(cfg["sample_seed"]))
    problems = sample_start_goal_pairs(
        grid, count, rng, min_separation=float(cfg["min_separation"])
    )

    algorithms = (
        ("Dijkstra", lambda s, g: dijkstra(cost_map, s, g, **common)),
        ("A*", lambda s, g: astar(cost_map, s, g, **common)),
        (f"Weighted A* (w={weight})", lambda s, g: weighted_astar(cost_map, s, g, weight=weight, **common)),
    )
    common = dict(drivable=grid.drivable, min_cell_cost=min_cost, collect_expanded=False)

    rows: list[dict[str, Any]] = []
    for index, (start, goal) in enumerate(problems):
        optimal = None
        for name, run in algorithms:
            result = run(start, goal)
            if optimal is None:
                optimal = result.cost
            rows.append({
                "pair": index,
                "algorithm": name,
                "start_x": start[0], "start_y": start[1],
                "goal_x": goal[0], "goal_y": goal[1],
                "success": result.success,
                "cost": result.cost,
                "optimal_cost": optimal,
                "cost_ratio": result.cost / optimal if optimal else float("nan"),
                "nodes_expanded": result.nodes_expanded,
                "runtime_ms": result.runtime_ms,
                "path_cells": 0 if result.path is None else len(result.path),
            })
    return pd.DataFrame(rows)


def write_grid_benchmark(frame: pd.DataFrame, path: Path | str | None = None) -> Path:
    """Write the benchmark rows to CSV and return the path."""
    out = Path(path) if path is not None else results_path(GRID_CSV)
    out.parent.mkdir(parents=True, exist_ok=True)
    frame.to_csv(out, index=False)
    return out


def summarise_grid(frame: pd.DataFrame) -> pd.DataFrame:
    """Per-algorithm summary used by the step-1 table in the report."""
    return frame.groupby("algorithm", sort=False).agg(
        pairs=("pair", "nunique"),
        mean_cost=("cost", "mean"),
        mean_cost_ratio=("cost_ratio", "mean"),
        max_cost_ratio=("cost_ratio", "max"),
        mean_nodes_expanded=("nodes_expanded", "mean"),
        max_nodes_expanded=("nodes_expanded", "max"),
        mean_runtime_ms=("runtime_ms", "mean"),
        max_runtime_ms=("runtime_ms", "max"),
    ).reset_index()
