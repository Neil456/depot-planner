"""Step 7 benchmark: the pure-Python step-1 search against the C++ core."""

from __future__ import annotations

import time
from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd

from depot_planner.config import load_config, results_path
from depot_planner.grid_astar.backend import extension_available
from depot_planner.grid_astar.search import search
from depot_planner.world.depot_maps import generate_depot, sample_start_goal_pairs

CPP_CSV = "cpp_benchmark.csv"


def compare_backends(
    pairs: int | None = None,
    config: dict[str, Any] | None = None,
    repeats: int = 3,
) -> pd.DataFrame:
    """Time both backends on the same problems; one row per (pair, algorithm).

    Raises if the C++ core is not built: the caller decides whether that is fatal.
    """
    if not extension_available():
        raise RuntimeError("the C++ core is not built")

    cfg = (config if config is not None else load_config("eval"))["grid_benchmark"]
    weight = float(load_config("grid")["search"]["weight"])
    count = int(pairs if pairs is not None else cfg["pairs"])

    grid = generate_depot(seed=int(cfg["map_seed"]))
    cost_map = grid.cost_map()
    min_cost = grid.min_cell_cost()
    rng = np.random.default_rng(int(cfg["sample_seed"]))
    problems = sample_start_goal_pairs(grid, count, rng, min_separation=float(cfg["min_separation"]))

    algorithms = (
        ("Dijkstra", "none", 1.0),
        ("A*", "octile", 1.0),
        (f"Weighted A* (w={weight})", "octile", weight),
    )
    common = dict(drivable=grid.drivable, min_cell_cost=min_cost, collect_expanded=False)

    rows: list[dict[str, Any]] = []
    for index, (start, goal) in enumerate(problems):
        for label, heuristic, algorithm_weight in algorithms:
            timings: dict[str, float] = {}
            results = {}
            for backend in ("python", "cpp"):
                best = float("inf")
                for _ in range(max(1, repeats)):
                    began = time.perf_counter()
                    result = search(cost_map, start, goal, heuristic=heuristic,
                                    weight=algorithm_weight, backend=backend, **common)
                    best = min(best, (time.perf_counter() - began) * 1000.0)
                timings[backend] = best
                results[backend] = result
            python_result, cpp_result = results["python"], results["cpp"]
            rows.append({
                "pair": index,
                "algorithm": label,
                "python_ms": timings["python"],
                "cpp_ms": timings["cpp"],
                "speedup": timings["python"] / timings["cpp"] if timings["cpp"] else float("nan"),
                "python_cost": python_result.cost,
                "cpp_cost": cpp_result.cost,
                "cost_difference": abs(python_result.cost - cpp_result.cost),
                "same_path": python_result.path == cpp_result.path,
                "nodes_expanded": python_result.nodes_expanded,
            })
    return pd.DataFrame(rows)


def write_cpp_benchmark(frame: pd.DataFrame, path: Path | str | None = None) -> Path:
    """Write the timing rows to CSV and return the path."""
    out = Path(path) if path is not None else results_path(CPP_CSV)
    out.parent.mkdir(parents=True, exist_ok=True)
    frame.to_csv(out, index=False)
    return out


def summarise_cpp(frame: pd.DataFrame) -> pd.DataFrame:
    """Per-algorithm means, plus whether the two backends agreed exactly."""
    return frame.groupby("algorithm", sort=False).agg(
        pairs=("pair", "nunique"),
        mean_python_ms=("python_ms", "mean"),
        mean_cpp_ms=("cpp_ms", "mean"),
        mean_speedup=("speedup", "mean"),
        max_cost_difference=("cost_difference", "max"),
        identical_paths=("same_path", "all"),
    ).reset_index()
