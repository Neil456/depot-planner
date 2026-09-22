"""Timing the Python reference against the C++ core, planner by planner.

One row per timed case. Each case is run ``repeats`` times on each backend and
the **median** is kept, because a mean is dragged around by the occasional
scheduling hiccup and a minimum flatters whichever backend gets luckier.

Everything a case needs — cost maps, drivable masks, cost-to-go fields, agent
timelines, distance fields — is built once, before the clock starts, so what is
timed is the search and nothing else. The same prepared inputs are handed to
both backends.

The equivalent C++-only suite is ``cpp/bench/bench_planners.cpp``, run with
``make cpp-bench``; it uses Google Benchmark and its own fixed scenarios, so it
measures the core without the binding layer.
"""

from __future__ import annotations

import statistics
import time
from pathlib import Path
from typing import Any, Callable, Sequence

import numpy as np
import pandas as pd

from depot_planner import core
from depot_planner.config import load_config, results_path
from depot_planner.grid_astar.search import search
from depot_planner.hybrid.search import plan_for_scenario
from depot_planner.spacetime import search as st
from depot_planner.world.depot_maps import generate_depot, sample_start_goal_pairs
from depot_planner.world.parking import PARKING_TYPES, generate_parking_scenario
from depot_planner.world.scenarios import SCENARIO_TYPES, generate_scenario

CPP_CSV = "cpp_benchmark.csv"

#: How many times each case is run on each backend before taking the median.
DEFAULT_REPEATS = 5

#: Cases per planner. Kept small for hybrid A*, where one Python plan can take
#: over a second.
DEFAULT_GRID_PAIRS = 40
DEFAULT_SPACETIME_SCENARIOS = 2
DEFAULT_PARKING_SCENARIOS = 2


def _median_ms(run: Callable[[], Any], repeats: int) -> tuple[float, Any]:
    """Median wall-clock of ``repeats`` runs, plus the last result."""
    timings: list[float] = []
    result = None
    for _ in range(max(1, repeats)):
        began = time.perf_counter()
        result = run()
        timings.append((time.perf_counter() - began) * 1000.0)
    return statistics.median(timings), result


def _row(planner: str, case: str, repeats: int, python: tuple[float, Any],
         cpp: tuple[float, Any], agree: bool, nodes: int) -> dict[str, Any]:
    python_ms, cpp_ms = python[0], cpp[0]
    return {
        "planner": planner,
        "case": case,
        "repeats": repeats,
        "python_ms": python_ms,
        "cpp_ms": cpp_ms,
        "speedup": python_ms / cpp_ms if cpp_ms else float("nan"),
        "agree": agree,
        "nodes_expanded": nodes,
    }


# ------------------------------------------------------------------ grid A*


def _grid_cases(pairs: int, repeats: int) -> list[dict[str, Any]]:
    cfg = load_config("eval")["grid_benchmark"]
    weight = float(load_config("grid")["search"]["weight"])
    grid = generate_depot(seed=int(cfg["map_seed"]))
    cost_map = grid.cost_map()
    min_cost = grid.min_cell_cost()
    drivable = grid.drivable
    rng = np.random.default_rng(int(cfg["sample_seed"]))
    problems = sample_start_goal_pairs(grid, pairs, rng, min_separation=float(cfg["min_separation"]))

    algorithms = (
        ("grid A* (Dijkstra)", "none", 1.0),
        ("grid A*", "octile", 1.0),
        (f"grid A* (weighted, w={weight})", "octile", weight),
    )
    common = dict(drivable=drivable, min_cell_cost=min_cost, collect_expanded=False)

    rows: list[dict[str, Any]] = []
    for index, (start, goal) in enumerate(problems):
        for label, heuristic, algorithm_weight in algorithms:
            def run(backend: str) -> Any:
                return search(cost_map, start, goal, heuristic=heuristic,
                              weight=algorithm_weight, backend=backend, **common)

            python = _median_ms(lambda: run("python"), repeats)
            cpp = _median_ms(lambda: run("cpp"), repeats)
            agree = (
                python[1].path == cpp[1].path
                and python[1].cost == cpp[1].cost
                and python[1].nodes_expanded == cpp[1].nodes_expanded
            )
            rows.append(_row(label, f"pair {index}", repeats, python, cpp, agree,
                             int(python[1].nodes_expanded)))
    return rows


# ------------------------------------------------------------ space-time A*


def _spacetime_cases(scenarios: int, repeats: int) -> list[dict[str, Any]]:
    offset = int(load_config("eval")["battery"]["seed_offset"])
    rows: list[dict[str, Any]] = []
    for scenario_type in SCENARIO_TYPES:
        for index in range(scenarios):
            scenario = generate_scenario(scenario_type, offset + index)
            grid = scenario.grid
            # Setup, excluded from the timing: the cost map, the drivable mask,
            # the cost-to-go field and the flattened agent timelines.
            cost_map = grid.cost_map()
            drivable = grid.drivable
            field = st.goal_cost_field(cost_map, drivable & np.isfinite(cost_map), scenario.goal)
            timelines = st.agent_timelines_for(scenario.agents)
            common = dict(start_time=0, min_cell_cost=grid.min_cell_cost(),
                          collect_expanded=False, heuristic_field=field,
                          agent_timelines=timelines,
                          max_time=min(700, int(scenario.time_limit)))

            def run(backend: str) -> Any:
                return st.plan(cost_map, drivable, scenario.start, scenario.goal,
                               scenario.agents, backend=backend, **common)

            python = _median_ms(lambda: run("python"), repeats)
            cpp = _median_ms(lambda: run("cpp"), repeats)
            agree = (
                python[1].cells == cpp[1].cells
                and python[1].times == cpp[1].times
                and python[1].cost == cpp[1].cost
                and python[1].nodes_expanded == cpp[1].nodes_expanded
            )
            rows.append(_row("space-time A*", f"{scenario_type} seed {offset + index}", repeats,
                             python, cpp, agree, int(python[1].nodes_expanded)))
    return rows


# ---------------------------------------------------------------- hybrid A*


def _hybrid_cases(scenarios: int, repeats: int) -> list[dict[str, Any]]:
    offset = int(load_config("eval")["battery"]["seed_offset"])
    rows: list[dict[str, Any]] = []
    for parking_type in PARKING_TYPES:
        for index in range(scenarios):
            scenario = generate_parking_scenario(parking_type, offset + index)
            # Setup, excluded from the timing: the distance field and the coarse
            # heuristic field are built and cached on the scenario here.
            scenario.distance_field()
            scenario.goal_distance_field()

            def run(backend: str) -> Any:
                return plan_for_scenario(scenario, collect_explored=False, backend=backend)

            python = _median_ms(lambda: run("python"), repeats)
            cpp = _median_ms(lambda: run("cpp"), repeats)
            agree = (
                python[1].poses == cpp[1].poses
                and python[1].cost == cpp[1].cost
                and python[1].nodes_expanded == cpp[1].nodes_expanded
            )
            rows.append(_row("hybrid A*", f"{parking_type} seed {offset + index}", repeats,
                             python, cpp, agree, int(python[1].nodes_expanded)))
    return rows


# ------------------------------------------------------------------- public


def compare_backends(
    pairs: int | None = None,
    repeats: int = DEFAULT_REPEATS,
    scenarios: int | None = None,
    planners: Sequence[str] | None = None,
) -> pd.DataFrame:
    """Time every planner on both backends; one row per case.

    Raises if the C++ core is not built: the caller decides whether that is fatal.
    """
    if not core.available():
        raise RuntimeError("the C++ core is not built")

    wanted = set(planners) if planners is not None else {"grid", "spacetime", "hybrid"}
    rows: list[dict[str, Any]] = []
    if "grid" in wanted:
        rows += _grid_cases(int(pairs if pairs is not None else DEFAULT_GRID_PAIRS), repeats)
    if "spacetime" in wanted:
        rows += _spacetime_cases(
            int(scenarios if scenarios is not None else DEFAULT_SPACETIME_SCENARIOS), repeats
        )
    if "hybrid" in wanted:
        rows += _hybrid_cases(
            int(scenarios if scenarios is not None else DEFAULT_PARKING_SCENARIOS), repeats
        )
    return pd.DataFrame(rows)


def write_cpp_benchmark(frame: pd.DataFrame, path: Path | str | None = None) -> Path:
    """Write the timing rows to CSV and return the path."""
    out = Path(path) if path is not None else results_path(CPP_CSV)
    out.parent.mkdir(parents=True, exist_ok=True)
    frame.to_csv(out, index=False)
    return out


def summarise_cpp(frame: pd.DataFrame) -> pd.DataFrame:
    """Per-planner medians, plus whether the two backends agreed on every case."""
    summary = frame.groupby("planner", sort=False).agg(
        cases=("case", "size"),
        repeats=("repeats", "max"),
        median_python_ms=("python_ms", "median"),
        median_cpp_ms=("cpp_ms", "median"),
        median_speedup=("speedup", "median"),
        median_nodes=("nodes_expanded", "median"),
        identical=("agree", "all"),
    ).reset_index()
    return summary
