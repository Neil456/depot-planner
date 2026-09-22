"""Parking battery: hybrid A* over every parking scenario type."""

from __future__ import annotations

import time
from pathlib import Path
from typing import Any, Sequence

import pandas as pd

from depot_planner.config import load_config, results_path
from depot_planner.hybrid.search import HybridResult, plan_for_scenario
from depot_planner.sim.car_collision import check_plan
from depot_planner.world.parking import PARKING_TYPES, generate_parking_scenario

PARKING_CSV = "battery_parking.csv"


def episode_row(scenario, result: HybridResult, verified: bool, detail: str) -> dict[str, Any]:
    return {
        "parking_type": scenario.name,
        "layout": scenario.layout,
        "seed": scenario.seed,
        "success": result.success,
        "planning_ms": result.runtime_ms,
        "nodes_expanded": result.nodes_expanded,
        "collision_checks": result.collision_checks,
        "path_length_m": result.path_length_m,
        "direction_switches": result.direction_switches,
        "reverse_length_m": result.reverse_length_m,
        "used_analytic": result.used_analytic,
        "cost": result.cost,
        "verified_collision_free": verified,
        "reason": result.reason if result.success else result.reason or "no path",
        "detail": detail,
    }


def run_parking_battery(
    parking_types: Sequence[str] = PARKING_TYPES,
    episodes_per_type: int | None = None,
    config: dict[str, Any] | None = None,
    verbose: bool = True,
) -> pd.DataFrame:
    """Plan every scenario and verify each plan with the independent checker.

    A plan the planner reports as successful but which the independent checker
    rejects is a bug, so it raises rather than being recorded as a success.
    """
    cfg = (config if config is not None else load_config("eval"))["battery"]
    episodes = int(episodes_per_type if episodes_per_type is not None else cfg["episodes_per_type"])
    offset = int(cfg["seed_offset"])

    rows: list[dict[str, Any]] = []
    began = time.perf_counter()
    for parking_type in parking_types:
        for index in range(episodes):
            seed = offset + index
            scenario = generate_parking_scenario(parking_type, seed)
            result = plan_for_scenario(scenario, collect_explored=False)
            if result.success:
                report = check_plan(scenario, result)
                if not report.ok:
                    raise AssertionError(
                        f"hybrid A* reported success on {parking_type} seed {seed} but the "
                        f"independent checker found: {report.describe()}"
                    )
                rows.append(episode_row(scenario, result, True, ""))
            else:
                rows.append(episode_row(scenario, result, False, result.reason))
        if verbose:
            print(f"  {parking_type:24s} {len(rows):4d} scenarios  ({time.perf_counter() - began:5.1f}s)")
    return pd.DataFrame(rows)


def write_parking_battery(frame: pd.DataFrame, path: Path | str | None = None) -> Path:
    out = Path(path) if path is not None else results_path(PARKING_CSV)
    out.parent.mkdir(parents=True, exist_ok=True)
    frame.to_csv(out, index=False)
    return out


def summarise_parking(frame: pd.DataFrame) -> pd.DataFrame:
    """Per parking type summary used by the report."""
    successful = frame[frame["success"]]
    grouped = frame.groupby("parking_type", sort=False)
    summary = grouped.agg(
        scenarios=("success", "size"),
        success_pct=("success", lambda s: 100.0 * s.mean()),
        mean_planning_ms=("planning_ms", "mean"),
        max_planning_ms=("planning_ms", "max"),
        mean_nodes_expanded=("nodes_expanded", "mean"),
    ).reset_index()
    solved = successful.groupby("parking_type", sort=False).agg(
        mean_direction_switches=("direction_switches", "mean"),
        mean_path_length_m=("path_length_m", "mean"),
        mean_reverse_length_m=("reverse_length_m", "mean"),
    ).reset_index()
    return summary.merge(solved, on="parking_type", how="left")


def parking_failures(frame: pd.DataFrame) -> pd.DataFrame:
    failed = frame[~frame["success"]].copy()
    return failed.sort_values(["parking_type", "seed"]).reset_index(drop=True)
